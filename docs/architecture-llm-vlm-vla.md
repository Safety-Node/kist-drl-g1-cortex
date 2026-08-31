# LLM + VLM + VLA 인지 아키텍처

자연어 명령("냉장고에서 오이 하나 가져다줘")이 로봇 관절 명령까지 내려가는
전체 경로와, 그 경로를 ROS 2 미들웨어 위에서 어떻게 나눴는지를 설명한다.

브랜치: `SYS-REQ-44-llm-vlm-vla-planning` · 대상 독자: 아키텍처 리뷰/토론
(구현 상세는 각 노드 docstring이 SSOT다 — 이 문서는 **왜 이렇게 나눴는가**를 다룬다)

---

## 1. 한 문장 요약

**LLM은 계획하고(장면을 못 봄), VLM은 장면을 보고 판정·번역하고(계획하지 않음),
VLA는 반사적으로 움직인다(텍스트+이미지→관절). 오케스트레이터는 이 셋을
ROS 토픽으로 묶는 open-loop 상태기계이며, 세 모델 어느 것도 서로를 직접
호출하지 않는다.**

```
                                    ┌─────────────────────── PC (cortex) ───────────────────────┐
발화 ──▶ stt_node ──transcript──▶ orchestrator_node ◀──Plan──── llm_node ──▶ (LLM API)
                                    │      │  ▲                PlanRequest
                                    │      │  └─VlaPrompt──── vlm_node ──▶ (VLM API)
                                    │      └──Subtask────────▶   │  ▲
                                    │         (goal_text,        │  └─task_progress── kist-vla-inference
                                    │          progress_gate)    └──Verdict            (GR00T policy)
                                    │                                                      ▲
                                    ├─ActionCmd(say)──▶ tts_node                           │
                                    ├─(stub)──────────▶ nav-planner                        │
                                    └─(stub)──────────▶ VLA prompt ────────────────────────┘
```

## 2. 자연어 → primitive 명령: 3단 번역

핵심 설계는 **한 번에 번역하지 않는 것**이다. 자연어에서 관절 명령까지
추상화 낙차가 너무 크므로, 서로 다른 능력을 가진 모델에 한 단씩 맡긴다.

| 단계 | 입력 → 출력 | 담당 | 왜 여기서 자르나 |
|---|---|---|---|
| ① 계획 | 발화 → sub-task 시퀀스(JSON) | llm_node | 상식·순서 추론은 LLM이 최강. 단 장면을 못 보므로 출력은 **추상 목표**까지만 |
| ② 접지(grounding) | 추상 목표 + 현재 프레임 → 장면 특화 명령문 | vlm_node | "문을 열어라"를 "**오른쪽** 문 손잡이 **아래에 노란 팁을 걸고** 당겨라"로. 장면을 보는 자만 쓸 수 있는 문장 |
| ③ 실행 | 명령문 + 이미지 + 관절상태 → 관절 궤적 | VLA (GR00T) | 학습된 반사. 텍스트 프롬프트가 곧 인터페이스 |

②가 이 아키텍처의 특징이다. LLM이 장면 묘사까지 하려면 프레임을 계속
LLM에 보내야 하고(느림, 비쌈), VLA에 추상 목표를 직접 주면 학습 분포
밖의 프롬프트가 된다. VLM이 중간에서 **계획의 언어를 정책의 언어로
번역**한다.

### 실행 예 (냉장고 시나리오, llm 모드)

```
"오이 가져다줘"
 └▶ llm_node: {sub_tasks: [approach_fridge(nav), open_door(vla, grounded),
                            grasp_cucumber(vla, grounded)]}
     └▶ orchestrator: open_door 진입
         ├─ speak "냉장고 문을 엽니다"            (on_create)
         ├─ Subtask{goal_text: "open the refrigerator door",
         │          progress_gate: 0.9}          → vlm_node
         │   └▶ VlaPrompt{"Open the right door of the refrigerator.
         │                 Hook the yellow tip attached to your right
         │                 hand under the door handle and pull."}
         ├─ vla connector 발사 (grounded 텍스트)   → VLA
         ├─ (VLA task_progress 0.0 → … → 0.92)
         └─ progress ≥ 0.9 부터 vlm이 장면 판정   → Verdict{passed} → 다음 sub-task
```

## 3. Sub-task 포맷 — 라우팅은 데이터다

LLM 출력 포맷을 **파일 시나리오(JSON5)와 동일한 스키마**로 강제했다.
어느 액추에이터로 보낼지(vla / navigation / tts / 조합)는 메시지 타입이나
코드 분기가 아니라 **sub-task 안의 hook 리스트**가 결정한다:

```json5
{
  name: "open_door",
  on_create: [{ speak: "냉장고 문을 엽니다." }],          // tts
  on_start:  [{ vla: { grounded: true, goal: "open the refrigerator door" } }],
  //          [{ navigation: "goal:refrigerator" }]       // nav 라우팅이면 이렇게
  //          [{ speak: "..." }, { vla: {...} }]          // tts+vla 조합 = 스텝 2개
  success:   { type: "vlm", check: "refrigerator door is open",
               timeout_s: 20, progress_gate: 0.9 },
  on_fail:   [{ speak: "문을 열지 못했습니다." }],
}
```

이 결정의 파급효과가 크다:

- **실행 엔진이 하나다.** LLM 플랜과 파일 시나리오가 같은 로더
  (`scenario_from_raw`)를 지나므로, LLM 환각(없는 라벨, 없는 criterion,
  goal 빠진 grounded 스텝)이 **파일 오타와 똑같은 코드에서, 실행 전에**
  거부된다. 검증기를 두 벌 쓰지 않는다.
- **모드 전환이 파라미터 한 줄이다.** `planner_mode: static | llm`
  (cortex_params.yaml). 데모 당일 LLM이 불안하면 static으로 내리면 되고,
  엔진·커넥터·criterion은 한 글자도 바뀌지 않는다.
- **라우팅 확장이 스키마 무변경이다.** 새 액추에이터 = 새 connector 등록
  + LLM 시스템 프롬프트에 라벨 한 줄. 메시지 정의 변경 없음.

## 4. 메시지 계약 (신규분)

| 토픽 | 타입 | 방향 | 비고 |
|---|---|---|---|
| `/cortex/llm/request` | `PlanRequest` | orch → llm | request_id 상관, latest-wins |
| `/cortex/llm/plan` | `Plan` | llm → orch | `scenario_json` = 위 스키마. `ok=false`+`not_a_command` = 방관자 발화 무시 경로 |
| `/cortex/active_subtask` | `Subtask`(+`goal_text`,`progress_gate`) | orch → vlm | 판정 대상 + 접지 요청을 한 메시지로 |
| `/cortex/vla/prompt` | `VlaPrompt` | vlm → orch | 접지 결과. orch가 vla connector로 중계 |
| `/cortex/vla/task_progress` | `std_msgs/Float32` | **vla-inference** → vlm | ⚠️ 외부 배선 필요 — §7 |
| `/cortex/critic/verdict` | `Verdict` | vlm → orch | 기존. progress_gate 미달 시 **침묵**(발행 안 함) |

주목할 규약 두 가지:

- **VlaPrompt를 vlm→VLA 직결이 아니라 orchestrator 경유로 했다.** 인지
  스택의 모든 액추에이터 명령은 오케스트레이터라는 단일 지점을 지난다 —
  preemption(새 명령/E-STOP 시 취소)을 한 곳에서 보장하기 위해서다. VLM이
  직접 팔을 움직일 수 있는 경로를 만들지 않는다.
- **progress gate 미달 시 Verdict를 "실패"가 아니라 무발행으로 했다.**
  침묵 = "아직 모름"이며, 시간 상한은 오케스트레이터의 timeout이 소유한다.
  판정자와 시계 소유자를 분리하는 기존 원칙 그대로다.

## 5. 오케스트레이터의 특별함 — ROS 미들웨어 위의 설계

발표 관점에서 이 노드의 요점은 "LLM을 붙였다"가 아니라 **모델을 신뢰하지
않는 실행기**라는 것이다.

1. **모든 모델 호출이 tick 루프 밖이다.** LLM(초 단위), VLM(≈1 s), VLA
   (GPU)는 각자의 노드/프로세스에서 돌고, 오케스트레이터 tick(10 Hz)은
   캐시된 최신 값만 읽는다(`latest_verdict`, `_prompt_text`). 지능이
   아무리 느려도 상태기계는 멈추지 않는다 — OM1 포크를 버린 이유였던
   "tick을 막는 LLM cortex"의 정반대.
2. **비동기 상관은 전부 id + latest-wins.** Plan은 request_id로, VlaPrompt와
   Verdict는 subtask_id로 짝을 맞추고, 늦게 도착한 응답은 조용히 버린다.
   서비스/액션의 대기 대신 토픽 + 상관 id를 쓴 것은 의도적이다: 어느 모델이
   죽어도 블로킹이 없고, timeout이라는 단일 안전망으로 수렴한다.
3. **실패는 스키마가 소유한다.** on_fail 메시지는 엔진이 아니라 시나리오
   (= LLM 플랜)가 갖는다. LLM에게 실패 시 말할 문장까지 계획하게 하는 것 —
   계획과 실패 대응이 같은 산출물에 있으니 리뷰가 한 번에 된다.
4. **미래 아키텍처 자리 예약.** Connector 추상화(stub인 nav/vla 포함),
   Criterion 다형성, `_NOT_PORTED` 명시 실패는 전부 "나중에 붙을 것"의
   자리를 코드에 잡아 둔 형태다. 새 능력은 기존 능력의 수정이 아니라
   등록으로 들어온다.

### 테스트 용이성 — 같은 구조의 다른 얼굴

경계마다 dummy를 꽂을 수 있게 되어 있어, **네트워크·GPU·로봇 없이 전체
루프가 돈다**:

| 계층 | 실물 | 대역 |
|---|---|---|
| STT | Google Cloud | `backend: dummy` |
| 계획 | LLM API | `backend: dummy` (키워드→고정 플랜, 새 메시지 표면 전부 커버) |
| 판정/접지 | VLM API | stub `_evaluate`(항상 False — 오판정 불가) / `_ground`(에코) |
| 팔 | GR00T+로봇 | vla connector stub (로그) |

CI는 실행 없이 데이터를 검증한다: 시나리오 스키마 검사기가 grounded 페이로드
형식·progress_gate 범위·on_fail 누락까지 로드 전에 잡고, 로더의 fail-fast가
같은 규칙을 런타임(LLM 플랜)에도 적용한다. "시나리오는 코드가 아니라
데이터"라는 원칙이 LLM 도입 후에도 유지되는 이유다.

## 6. 미래 확장 (토론 포인트)

지금 구조에서 **엔진 수정 없이** 가능한 것과, 구조 변경이 필요한 것을 구분한다.

**엔진 무변경 (등록/프롬프트만):**
- 새 액추에이터(예: 그리퍼 단독, 표정) — connector 등록 + 프롬프트 라벨 추가
- 새 성공 판정(uwb_pose, joint_state) — criterion 등록 (`_NOT_PORTED`에 자리 있음)
- LLM 백엔드 교체/로컬화 — llm_node backend 추가

**작은 구조 변경:**
- **hybrid 모드** — static 트리거 우선 매칭 + 미스 시 LLM fallback.
  `_on_transcript` 분기 하나. 데모 안정성(고정 시나리오)과 유연성(자유 발화)의
  절충으로 유력.
- **실패 시 재계획** — 지금은 on_fail 후 idle. Verdict.reason + 실패한 플랜을
  llm_node에 되돌려 수정 플랜을 받는 루프. PlanRequest에 context 필드 추가면 됨.
- **VLM 주기 접지 갱신** — 지금은 sub-task당 1회 접지. 장면이 크게 변하면
  VlaPrompt를 재발행하는 것으로 확장 가능 (orchestrator는 이미 최신 프롬프트만
  쓰므로 수신측 변경 불요).

**큰 구조 변경 (별도 논의):**
- **closed-loop 전환** — 현재 open-loop(발사 후 확인 없음)는 의도된 단순화.
  Gearsonic에 CommandStatus가 생기면 preemption이 핸드셰이크로 바뀐다
  (README의 계획된 방향).
- **대화 상태/블랙보드** — multi-turn("아니 왼쪽 거") 지원은 transcripts
  버퍼가 아니라 명시적 대화 상태가 필요.
- **skill library** — LLM이 sub-task를 zero-shot 생성하는 대신, 검증된
  sub-task 조각을 검색·조합. 플랜 품질의 상한을 끌어올리는 방향.

## 7. 외부 레포 배선 (이 브랜치 밖의 일)

| 레포 | 필요한 일 | 현황 |
|---|---|---|
| kist-vla-inference | `task_progress`를 `/cortex/vla/task_progress`(Float32)로 발행 | 정책은 이미 출력 중 — `src/vla/runner.py`의 `_run_inference`가 chunk에서 **버리고 있음**. 발행 한 줄 추가면 됨 |
| kist-vla-inference | VLA 프롬프트 수신 — 지금은 시작 시 고정 `config.prompt` | VlaPrompt(또는 ActionCmd) 구독으로 runtime prompt 교체 필요 (ICD-70 gap과 같은 계열) |
| kist-gearsonic-inference | nav/vla ActionCmd 수신부 (ICD-70) | 미구현 gap — 기존 이슈 그대로 |

progress 발행이 없는 동안의 동작은 안전한 방향으로 죽는다: progress_gate가
걸린 sub-task는 판정이 영원히 유보되고 timeout으로 실패한다(오판정 없음).

---

*문서 위치: `docs/architecture-llm-vlm-vla.md` · 갱신 책임: 이 경로를 바꾸는 PR*

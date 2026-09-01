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
                                    │   │  ▲  ▲                PlanRequest
                                    │   │  │  └─VlaPrompt / PreconditionReport
                                    │   │  │        ▲
                                    │   │  └─Verdict┴─── vlm_node ──▶ (VLM API)
                                    │   └──Subtask / VerdictRequest──▶ (평소 유휴,
                                    │                                   요청 시에만 동작)
                                    │  ◀──CommandStatus(완료 보고)── nav/vla 모듈 ⚠️미배선
                                    ├─ActionCmd(say)──▶ tts_node
                                    ├─(stub)──────────▶ nav-planner ──┐
                                    └─(stub)──────────▶ VLA prompt ───┴▶ kist-vla-inference
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
         ├─ Subtask{goal_text: "open the refrigerator door"} → vlm_node
         │   └▶ VlaPrompt{"Open the right door of the refrigerator.
         │                 Hook the yellow tip attached to your right
         │                 hand under the door handle and pull."}
         ├─ vla connector 발사 (grounded 텍스트)   → VLA. vlm은 유휴로 복귀
         ├─ (VLA 동작 중 — orch는 완료 신호 대기, timeout 상한)
         ├─ CommandStatus{source: vla, ok} ⚠️미배선  → orch
         └─ VerdictRequest → vlm 1회 판정 → Verdict{passed} → 다음 sub-task
                                            (failed면 즉시 on_fail)
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
  success:   { type: "vlm", check: "refrigerator door is open", timeout_s: 20 },
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
| `/cortex/active_subtask` | `Subtask`(+`goal_text`,`precondition_check`) | orch → vlm | 진입 통보: 접지·전제 판정 요청 |
| `/cortex/vla/prompt` | `VlaPrompt` (+`precondition_met`, `precondition_why`) | vlm → orch | 접지 결과 + 전제 판정. orch가 vla connector로 중계 |
| `/cortex/command_status` | `CommandStatus` | **nav/vla 모듈** → orch | 동작 완료/실패 보고. ⚠️ 발행측 외부 미배선 — §7 |
| `/cortex/critic/request` | `VerdictRequest` | orch → vlm | 완료 신호가 모이면 **1회** 판정 요청 |
| `/cortex/critic/verdict` | `Verdict` | vlm → orch | 요청당 정확히 1건. fail → 즉시 on_fail |

주목할 규약 두 가지:

- **VlaPrompt를 vlm→VLA 직결이 아니라 orchestrator 경유로 했다.** 인지
  스택의 모든 액추에이터 명령은 오케스트레이터라는 단일 지점을 지난다 —
  preemption(새 명령/E-STOP 시 취소)을 한 곳에서 보장하기 위해서다. VLM이
  직접 팔을 움직일 수 있는 경로를 만들지 않는다.
- **precondition = "이 sub-task를 지금 실행할 수 있는가"** — 모든 sub-task에 쓸 수
  있고, 게이트 규칙은 **통일**돼 있다: precondition이 선언되면 판정 도착까지
  **on_start 전체가 유예**된다. combo sub-task(`on_start: [{navigation}, {vla}]`)의
  nav 스텝도 판정 전에는 나가지 않는다 — "정책상 움직이면 안 되는" 상태에서
  어떤 액추에이터도 출발하지 않는 것이 이 게이트의 계약이다.
  판정 회신 경로는 둘: grounded vla 스텝이 있으면 goal_text를 접지하는 **같은
  VLM 호출**에 합승해 VlaPrompt가 판정 겸 프롬프트로 돌아온다(프롬프트가 필수
  데이터라 fail-open이 불가능하므로 대기 상한 = sub-task의 timeout_s). 없으면
  (nav 등) **전용 판정**이 PreconditionReport로 돌아오고, `precondition_timeout_s`
  (3s) 안에 안 오면 **fail-open으로 출발**한다(vlm_node가 죽어도 VLM이 필요 없는
  이동이 발이 묶이면 안 된다). 판정 에러도 fail-open(met=true).
  timeout 의미: precondition이 있는 sub-task는 게이트 대기가 `_t0`(동작 시계)
  이전에 끝나므로 timeout_s가 순수 동작 시간을 예산한다. 없는 grounded
  sub-task는 기존대로 grounding+동작을 합산한다.
  이 게이트는 최적화지 안전 인터록이 아니며(그건 E-STOP), 동적 장애물(사람
  난입)의 연속 감시는 nav 스택·safety 레이어 몫 — start-time 스냅샷은 그
  대체물이 아니다. 기본은 **shadow 모드**(`precondition_enforce: false`) —
  unmet을 경고 로그로만 남기고 진행하며, shadow 로그와 실제 결과의 상관이
  확인된 뒤에만 enforce로 승격한다.
- **판정은 on-demand, one-shot이다.** vlm_node에 주기 판정 루프가 없다 —
  sub-task의 모든 동작이 완료 신호(CommandStatus)를 보내면 orchestrator가
  VerdictRequest를 정확히 1회 보내고, Verdict 1건으로 끝난다(passed → 진행,
  failed → 즉시 on_fail). progress_gate는 제거됐다: "동작 중간 장면 오판 방지"
  문제를 임계값 비교로 우회하는 대신, "완료 주장 → 그때 검증"이라는 정공법으로
  풀었다. VLM 비용은 ~1회/초 → ~1회/sub-task로 줄고, VLA→VLM으로 orchestrator를
  우회하던 task_progress 사이드 채널이 사라져 모든 조율이 다시 orchestrator를
  지난다. 완료 신호가 영영 안 오는 동작은 여전히 timeout이 잡는다.

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
형식·제거된 progress_gate 사용·on_fail 누락까지 로드 전에 잡고, 로더의 fail-fast가
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
- ~~precondition 일반화 (nav 포함)~~ — **구현됨** (§4 참고). nav 등 비-grounded
  sub-task도 precondition을 선언하면 전용 VLM 판정(PreconditionReport)이
  on_start를 게이트한다(fail-open 3s). 남은 것: 실물 VLM 백엔드가 붙은 뒤
  shadow 데이터로 판정 정확도 검증 → enforce 승격 판단.

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
| kist-vla-inference | 동작 완료 시 `CommandStatus{source: "vla", ok}` 발행 (`/cortex/command_status`) | 미배선 — 정책의 task_progress 출력을 내부 임계값과 비교해 완료를 판단하면 됨 (임계값 소유권은 VLA 측) |
| kist-vla-inference | VLA 프롬프트 수신 — 지금은 시작 시 고정 `config.prompt` | VlaPrompt(또는 ActionCmd) 구독으로 runtime prompt 교체 필요 (ICD-70 gap과 같은 계열) |
| kist-gearsonic-inference | nav 명령 수신부 (ICD-70) + 도착 시 `CommandStatus{source: "navigation", ok}` 발행 | 미구현 gap — 기존 이슈 + 완료 보고 추가 |

완료 신호가 없는 동안의 동작은 안전한 방향으로 죽는다: 판정 요청이 영영
발사되지 않으므로 vlm criterion sub-task는 timeout으로 실패한다(오판정 없음).

---

*문서 위치: `docs/architecture-llm-vlm-vla.md` · 갱신 책임: 이 경로를 바꾸는 PR*

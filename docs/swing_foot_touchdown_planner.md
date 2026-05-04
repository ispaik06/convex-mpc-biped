# Swing Foot Touchdown Planner

이 문서는 `SwingFootPlanner`가 swing foot touchdown target, 즉 빨간 touchdown marker 위치를 어떻게 계산하는지 설명한다.
현재 구현은 `body_velocity_half_stance` 기반 touchdown geometry를 사용하고,
추가로 touchdown target을 `fixed` 또는 `realtime` 모드로 갱신할 수 있다.
이전의 `legacy_com_yaw_corrected` 방식은 제거되었다.

keyboard로 들어오는 연속 명령은 controller 내부 `user_command_filter`를 거친 뒤에만 사용된다.
walking의 `x_dot / y_dot / psi_dot`뿐 아니라 standing의 `z_dot / roll / pitch`도 같은 경로를 탄다.

## 1. 목표

보행 중 touchdown target은 다음 조건을 만족해야 한다.

1. 전진하면 다음 발도 계속 앞으로 짚어야 한다.
2. 후진하면 다음 발도 계속 뒤로 짚어야 한다.
3. 좌/우 이동하면 footprint 전체가 좌/우로 이동해야 한다.
4. 제자리 회전하면 발 위치는 몸 중심 주변에서 회전 배치되어야 한다.
   이때 `turn_in_place_*` bias가 켜지면 desired body marker 자체를 약간 앞으로 밀어서
   forward tip을 줄인다.
5. 정지가 완료되면 양발 평균 xy가 body desired marker, 즉 `debug_body_target`의 xy와 맞아야 한다.
6. 정지 진입 순간에는 현재 COM 속도를 보고 touchdown target을 desired marker보다 진행 방향 쪽으로 조금 더 둬서 브레이크를 건다.

핵심 설계 판단은 다음과 같다.

```text
desired body marker를 발에 끌어맞추지 않고,
발 touchdown target을 desired body marker에 맞춘다.
```

이유는 MPC의 reduced-body reference가 `_bodyTarget`에서 만들어지기 때문이다.
정지 시 desired body marker가 앞에 있는데 발만 뒤에 남으면, MPC는 COM을 marker로 보내려 하고 support polygon은 뒤에 남아서 넘어지기 쉽다.
따라서 정지 시에는 다음 swing foot을 marker 기준 footprint 위치로 보내고, 한두 step 뒤 양발 평균이 marker xy와 맞게 만드는 편이 자연스럽다.

## 2. 사용하는 좌표계

World frame `W`:

- 시뮬레이션/world 기준 좌표계.
- 최종 touchdown target은 world position `target_W`로 나온다.

Body yaw frame `B`:

- roll/pitch는 무시하고 yaw만 body heading에 맞춘 frame.
- user command `[x_dot, y_dot]`는 이 frame 기준 명령이다.
- 발 좌우 offset도 이 frame에서 기억한다.

현재 코드에서 body yaw는 실제 state yaw가 아니라 SRB desired yaw를 우선 사용한다.

```cpp
_swingFootPlanner->setBodyTargetWorld(_bodyTarget.position_W,
                                      _bodyTarget.euler_W[2]);
```

즉 touchdown 위치와 foot yaw는 가능한 한 MPC가 따라가려는 desired body marker 기준으로 계산된다.

## 3. 처음 저장하는 nominal foot offset

planner가 reset된 뒤 처음 호출되면 현재 양발 평균을 계산한다.

```text
footCenter_W = (leftFoot_W + rightFoot_W) / 2
```

각 발에 대해서 현재 발 위치가 이 center에서 얼마나 떨어져 있는지를 body yaw frame으로 변환한다.

```text
nominalFootOffset_B[leg] = Rz(yaw_des)^T * (foot_W[leg] - footCenter_W)
```

그 다음 x와 z는 0으로 만든다.

```text
nominalFootOffset_B[leg].x = 0
nominalFootOffset_B[leg].z = 0
```

그래서 offset은 사실상 "왼발은 center에서 y가 얼마", "오른발은 center에서 y가 얼마"만 기억한다.
이렇게 해야 전진/후진 시에는 양발이 같은 전후 위치 라인으로 정렬될 수 있고, 좌우 간격은 보존된다.

예:

```text
왼발 offset_B  = [0, +0.08, 0]
오른발 offset_B = [0, -0.08, 0]
```

## 4. Preview time

touchdown target은 지금 발 위치가 아니라 "landing 시점에 몸이 어디에 있을지"를 보고 잡는다.
그 시간 간격을 preview time이라고 부른다.

```text
previewTime = (0.5 + body_velocity_half_stance_offset) * stanceTime
```

기본적인 직관은 다음과 같다.

- `0.5 * stanceTime`: 다음 stance 구간의 중간 정도를 기준으로 발을 놓는다.
- `body_velocity_half_stance_offset`: 튜닝용 offset이다.
  - 0이면 half-stance preview.
  - 양수면 더 먼 미래를 보고 더 앞에 둔다.
  - 음수면 더 가까운 미래를 보고 덜 멀리 둔다.

현재 YAML에서 이 값이 0.5라면:

```text
previewTime = (0.5 + 0.5) * stanceTime = 1.0 * stanceTime
```

즉 touchdown target은 한 stance time 뒤의 desired footprint center를 보게 된다.

## 5. Swing Foot Yaw

스윙 중 발 자체의 yaw target은 다음처럼 잡는다.

```text
yaw0 = 현재 SRB desired yaw
swingFootYawTarget = yaw0 + swing_foot_yaw_lead_scale * psi_dot * previewTime
```

여기서 `psi_dot`은 user command의 yaw rate다. 실제 구현에서는 이 lead 항에
`swing_foot_yaw_lead_scale`를 곱한다.

예를 들어 scale이 `1.5`면, 같은 `psi_dot`에 대해 swing foot yaw를 50% 더 앞서게 잡는다.

이 yaw는 두 군데에 쓰인다.

1. swing foot attitude PD의 yaw target
2. debug visualization의 swing foot yaw marker

따라서 제자리 회전 중에는 발 자체가 더 강하게 desired yaw를 따라간다.
이 값은 touchdown 위치를 직접 바꾸지 않는다.

## 6. Translation yaw와 0.5의 의미

translation step은 다음 식으로 계산한다.

```text
translationYaw = yaw0 + 0.5 * psi_dot * previewTime
step_W = Rz(translationYaw) * [x_dot, y_dot, 0] * previewTime
```

왜 `0.5`가 들어가나?

preview 구간 동안 body yaw가 `yaw0`에서 `yaw0 + psi_dot * previewTime`까지 계속 변한다고 생각하면,
속도 명령 `[x_dot, y_dot]`를 world로 바꿀 때 yaw도 시간에 따라 변한다.

정확히는 다음 적분이다.

```text
step_W = integral from 0 to T of Rz(yaw0 + psi_dot * t) * v_cmd_B dt
```

하지만 매번 삼각함수 적분을 쓰면 복잡하고, 짧은 preview 구간에서는 중간 yaw를 쓰는 근사가 충분히 직관적이다.

```text
중간 yaw = yaw0 + psi_dot * T / 2
```

그래서 `translationYaw`에 `0.5`가 붙는다.
이것은 "preview 시간 동안 몸이 회전하는 중간 방향으로 이동량을 world에 투영한다"는 뜻이다.

특히 `psi_dot = 0`이면:

```text
translationYaw = yaw0
```

그래서 일반적인 직선 이동 식과 완전히 같아진다.

## 7. 최종 touchdown target 식

현재 구현의 핵심 식은 다음과 같다.

```text
currentCenter_W = bodyTarget_W
futureCenter_W = currentCenter_W + step_W
target_W = futureCenter_W + Rz(yaw0) * nominalFootOffset_B[leg]
target_W.z = kSwingFootTargetZ
```

코드로는 다음 구조다.

```cpp
const double previewTime = touchdownPreviewTime();
const double yaw0 = bodyYawTargetWorld();
const double translationYaw_W = yaw0 + 0.5 * psi_dot * previewTime;
const Vec3<double> step_W = Rz(translationYaw_W) * v_body_cmd * previewTime;

Vec3<double> target =
    currentCenter_W + step_W + Rz(yaw0) * _nominalFootOffsets_B[legIndex];
```

`currentCenter_W`는 보통 `_bodyTarget.position_W`다.
만약 body target이 아직 planner에 들어오지 않았다면 내부에 저장된 마지막 footprint center를 fallback으로 쓴다.

## 8. 왜 previous foot target을 그대로 쓰지 않는가

이전 방식처럼 "직전 touchdown target을 다음 발의 p_init으로 그대로 사용"하면 이런 문제가 생긴다.

```text
왼발 touchdown  = [x=0.10, y=+0.08]
다음 오른발 p_init = 왼발 touchdown
오른발 touchdown = [x=0.20, y=+0.08]
```

오른발이 왼발 라인으로 들어오면서 좌우 발 간격이 깨진다.

그래서 현재 구현은 "발 위치 자체"를 다음 기준으로 쓰지 않고, 다음처럼 center를 복원한다.

```text
footprintCenter = target_W - Rz(yaw0) * nominalFootOffset_B[leg]
```

그러면 왼발 target으로부터도 center를 알 수 있고, 오른발 target으로부터도 같은 center를 알 수 있다.
다음 발은 이 center와 자기 offset으로 계산되므로 좌우 간격이 유지된다.

## 9. User command별 동작

### 9.1 정지: raw command가 zero로 들어온 경우

정지 진입은 `space`로 raw target이 zero가 되는 순간 시작된다.
다만 그 뒤의 `x_dot / y_dot / psi_dot`는 controller 내부 `user_command_filter`를 거치므로,
몸통 target과 MPC reference는 즉시 0으로 끊기지 않고 천천히 따라간다.
즉 body desired marker도 filtered command를 따라 서서히 감속한다.

현재 구현에서는 감속/정지용 capture-point braking offset을 touchdown 식에 다시 더한다.
단, `space`가 들어와도 steady walking 중에는 touchdown target이 그대로 유지되고,
filtered planar command가 줄어들거나 거의 0이 될 때만 braking offset을 더한다.

관련 YAML 키 `stop_capture_point_gain`, `stop_capture_point_max_offset`, `stop_velocity_deadband`는 braking offset 계산에 사용된다.

### 9.2 전진: x_dot > 0, y_dot = 0, psi_dot = 0

```text
step_W = Rz(yaw0) * [x_dot, 0, 0] * previewTime
target_W = bodyTarget_W + step_W + Rz(yaw0) * offset_B[leg]
```

결과:

- future footprint center가 body heading 앞쪽으로 이동한다.
- 왼발/오른발은 같은 future center 주변의 좌우 offset 위치로 간다.

### 9.3 후진: x_dot < 0, y_dot = 0, psi_dot = 0

```text
step_W = Rz(yaw0) * [negative, 0, 0] * previewTime
```

결과:

- future footprint center가 body heading 뒤쪽으로 이동한다.
- 발은 뒤쪽 target으로 잡힌다.

### 9.4 좌/우 이동: y_dot != 0

```text
step_W = Rz(yaw0) * [0, y_dot, 0] * previewTime
```

결과:

- `y_dot > 0`이면 body left 방향으로 footprint center가 이동한다.
- `y_dot < 0`이면 body right 방향으로 footprint center가 이동한다.
- 각 발의 nominal left/right offset은 여전히 유지된다.

### 9.5 제자리 회전: x_dot = 0, y_dot = 0, psi_dot != 0

```text
turnBias_W = Rz(yaw0) * [d_turn, 0, 0]
target_W = bodyTarget_W + turnBias_W + Rz(yaw0) * offset_B[leg]
```

결과:

- desired body marker는 turn bias 만큼 body frame 전방으로 이동한다.
- 발 위치는 그 biased marker 주변에서 회전 배치된다.
- swing foot yaw target은 별도의 `swing_foot_yaw_lead_scale`를 사용해서 더 강하게 desired yaw를 따라간다.

### 9.6 이동하면서 회전: x_dot/y_dot != 0, psi_dot != 0

```text
step_W = Rz(yaw0 + 0.5 * psi_dot * previewTime) * v_cmd_B * previewTime
target_W = bodyTarget_W + step_W + Rz(yaw0) * offset_B[leg]
```

결과:

- center 이동은 preview 구간의 중간 yaw를 기준으로 근사한다.
- 발 좌우 배치는 body desired yaw를 기준으로 한다.
- 따라서 몸이 회전하며 이동할 때도 발이 미래 heading에 맞춰 놓인다.
- raw command가 zero로 들어가면 filtered command가 줄어드는 동안 braking offset이 켜지고, command가 다시 steady해지면 꺼진다.

## 10. Red marker와 균형

`debug_body_target`은 reduced-body COM desired marker다.
MPC reference는 이 marker를 기반으로 만들어진다.

정지 시 target 식이 다음처럼 되므로:

```text
leftTarget_W  = bodyTarget_W + Rz(yaw0) * leftOffset_B
rightTarget_W = bodyTarget_W + Rz(yaw0) * rightOffset_B
```

그리고 offset이 초기 양발 평균 기준으로 저장되어 있다면:

```text
(leftTarget_W.xy + rightTarget_W.xy) / 2 ~= bodyTarget_W.xy
```

즉 정지 후 support footprint center가 desired body marker 아래로 들어오게 된다.

## 11. 현재 구현에서 의도적으로 하지 않는 것

touchdown target 갱신 모드는 두 가지다.

```yaml
swing:
  touchdown_target_update_mode: fixed     # swing 시작 시 1회 계산
```

- 현재는 braking offset을 매 tick 다시 계산하지 않는다.
- turn-in-place bias도 매 tick 누적하지 않는다. filtered `psi_dot`가 deadband를 넘는 동안만
  `turn_in_place_support_offset_gain`과 `turn_in_place_support_offset_max`로 제한된 forward bias를
  desired body marker에 더한다.
- `fixed`: swing이 시작될 때 touchdown target을 한 번 잡고, 평상시에는 그 swing 동안 유지한다.
- command가 줄어들거나 stop 상태로 들어가는 순간에만 braking offset을 포함해 target을 다시 잡는다.
- 현재 touchdown target 계산은 fixed 방식만 사용한다.

- `space`로 raw target이 zero가 되더라도, command가 실제로 줄어드는 구간이 아니면 braking offset은 켜지지 않는다.
- touchdown target은 steady walking 중에는 유지되다가, command가 감속/정지로 들어갈 때만 braking offset을 포함해 다시 계산된다.
- pure turn-in-place일 때는 `turn_in_place_psi_dot_deadband`를 넘는 `psi_dot`만으로 turn bias가 켜진다.

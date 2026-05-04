# MPC Frame Convention And Cost Weighting

이 문서는 reduced-body Convex MPC에서 world frame과 body-yaw frame을 어떻게 나눠 써야 하는지,
그리고 현재 yaw 회전 중 불안정성이 왜 cost weight frame 문제에서 나올 수 있는지 정리한다.

목표는 나중에 구현할 때 다음 방향을 기준으로 삼는 것이다.

1. MPC dynamics와 target position은 world frame을 유지한다.
2. user command, foot nominal offset, capture offset, cost 의미는 body-yaw frame 기준으로 해석한다.
3. QP convexity를 유지하기 위해 cost 변환은 reference yaw 기반의 선형 변환으로 넣는다.

## 1. 현재 구현 요약

현재 reduced-body MPC state는 대략 다음 순서를 사용한다.

```text
x = [
  roll, pitch, yaw,
  px, py, pz,
  omega_x, omega_y, omega_z,
  vx, vy, vz,
  g
]
```

현재 코드상 frame 의미는 다음과 같다.

- `x0[0:3]`: torso orientation에서 얻은 roll/pitch/yaw. yaw는 world heading이다.
- `x0[3:6]`: reduced-body COM position in world.
- `x0[6:9]`: torso angular velocity in world.
- `x0[9:12]`: reduced-body COM velocity in world.
- `X_ref`: body target world position과 yaw를 seed로 만들어지는 world-frame reference.
- user command `[x_dot, y_dot, z_dot]`: body-yaw frame command.
- `ReferenceTrajectory`: `Rz(psi_ref) * u_cmd_B`로 world velocity reference를 만든다.
- `MPCFormulation`: yaw-aligned inertia를 world로 회전해서 world-frame dynamics matrix를 만든다.
- `ConvexMPC::buildQP`: `stateError = A_qp * x0 - X_ref`를 world coordinate 그대로 만들고, 고정 diagonal state weight를 곱는다.

핵심 문제는 마지막 항이다.

예를 들어 config에서 `px`와 `py` weight가 다르면, 이 값은 의도상 body forward/lateral tracking weight일 가능성이 크다.
하지만 현재는 world x/y에 고정된다. 로봇이 yaw 회전하면 body forward/lateral 방향이 world x/y와 달라지므로,
cost의 물리적 의미가 yaw에 따라 바뀐다.

## 2. 왜 yaw 회전 중 넘어질 수 있는가

현재 cost가 다음 형태라고 보면 된다.

```text
e_W = x_pred_W - x_ref_W
J = || Q_world_fixed * e_W ||^2
```

`Q_world_fixed`가 diagonal이고 `Q_px != Q_py`, `Q_vx != Q_vy`, `Q_roll != Q_pitch`이면,
weight는 world axis에 고정된다.

이 경우 yaw가 90도 바뀌면 다음 문제가 생긴다.

- "body lateral error를 강하게 억제한다"가 아니라 "world y error를 강하게 억제한다"가 된다.
- "body forward velocity를 추종한다"가 아니라 "world x velocity를 추종한다"에 가까워진다.
- foot placement는 body yaw 기준으로 돌고 있는데, torso cost는 world axis 기준으로 남는다.
- stance foot support geometry와 body tracking objective의 의미가 회전 중 어긋날 수 있다.

현재 설정 예:

```text
MIT humanoid walking:
  px = 5000
  py = 47300
  vx = 1000
  vy = 1000

H1/G1 walking:
  px = 2000
  py = 10700
  vx = 500
  vy = 550
```

`py`가 `px`보다 큰 것은 보통 body lateral direction을 더 강하게 잡으려는 튜닝이다.
하지만 현재 구조에서는 yaw가 바뀌면 lateral 방향이 아니라 world y 방향을 강하게 잡는다.

## 3. 권장 frame 분리

### World Frame으로 유지할 것

다음 값들은 world frame을 유지하는 것이 좋다.

- reduced-body COM position target `p_des_W`
- reduced-body yaw target `psi_des_W`
- foot touchdown target `foot_des_W`
- contact point offset `r = foot_W - com_W`
- MPC dynamics `A`, `B`
- contact force and moment decision variable, 현재 구조 기준 `F_W`, `M_W`
- gravity
- terrain/world position
- debug marker position

이유:

- foot contact와 COM 위치는 실제 world geometry와 직접 연결된다.
- dynamics matrix는 현재 이미 world coordinate 기반으로 구성돼 있다.
- target marker와 visual debug도 world frame이 직관적이다.
- 전체 formulation을 body frame으로 옮기면 foot contact offset, inertia, force constraints까지 같이 재정의해야 해서 변경 범위가 커진다.

### Body-Yaw Frame으로 해석할 것

다음 값들은 body-yaw frame 기준 의미를 가져야 한다.

- keyboard/user planar command `[x_dot, y_dot]`
- nominal foot offset
- swing touchdown preview translation
- stopping/capture offset
- MPC planar position tracking cost의 forward/lateral 의미
- MPC planar velocity tracking cost의 forward/lateral 의미
- 가능하면 roll/pitch 또는 angular velocity의 horizontal-axis cost 의미

여기서 body-yaw frame은 full body frame이 아니라 yaw만 반영한 frame이다.
roll/pitch까지 포함한 body frame을 쓰면 지면 기준 vertical과 tilt가 섞여서 reduced-body MPC에서는 오히려 해석이 복잡해진다.

## 4. 권장 MPC Cost 형태

전체 state와 dynamics는 world frame으로 유지하되, cost error만 reference yaw 기준 body-yaw frame으로 회전시킨다.

현재 형태:

```text
e_W = A_qp * x0 + B_qp * u - X_ref
J = || Q * e_W ||^2 + || R * u ||^2
```

권장 형태:

```text
e_body_yaw = T_k(psi_ref_k) * e_W
J = sum_k || Q_body * e_body_yaw_k ||^2 + || R * u ||^2
```

여기서 `T_k`는 step `k`마다 달라지는 state error transform이다.
`psi_ref_k`는 decision variable이 아니라 reference trajectory의 yaw를 사용한다.
따라서 QP는 여전히 convex quadratic problem으로 남는다.

현재 코드의 `stateWeight`는 sqrt-weight가 아니라 quadratic cost matrix처럼 쓰인다.
따라서 구현에서는 다음 cost matrix를 assembly에 넣는다.

```text
Q_world_k = T_k^T * Q_body * T_k
J_state_k = e_W_k^T * Q_world_k * e_W_k
```

평면 position error에 대해서는 다음과 같다.

```text
e_p_W = p_pred_W - p_ref_W
e_p_B = Rz(psi_ref)^T * e_p_W
```

평면 velocity error도 동일하다.

```text
e_v_W = v_pred_W - v_ref_W
e_v_B = Rz(psi_ref)^T * e_v_W
```

angular velocity도 world 표현이면 yaw 기준으로 회전해서 body-yaw 의미의 roll-rate/pitch-rate cost를 줄 수 있다.

```text
e_omega_W = omega_pred_W - omega_ref_W
e_omega_B = Rz(psi_ref)^T * e_omega_W
```

orientation error는 현재 state가 Euler `[roll, pitch, yaw]`이므로 엄밀한 SO(3) error는 아니다.
그래도 작은 roll/pitch 조건에서는 아래 근사가 실용적이다.

```text
e_rpy_W = rpy_pred - rpy_ref
e_rp_B_xy = Rz(psi_ref)^T * [e_roll, e_pitch, 0]
e_yaw = wrapToPi(yaw_pred - yaw_ref)
```

단, yaw error wrapping은 linear QP 예측식 안에서 직접 넣기 어렵다.
현재 yaw가 큰 폭으로 튀지 않는다는 가정이면 기존 차이를 쓰고, 필요하면 `X_ref` yaw를 현재 yaw 근처로 unwrap하는 방식이 더 안전하다.

## 5. 구현 포인트

가장 작은 변경은 `ConvexMPC::buildQP()`의 weighted assembly만 수정하는 것이다.

현재는 step마다 다음 형태다.

```cpp
_weightedB.middleRows(stateOffset, 13).noalias() =
    stateWeight * B_qp.middleRows(stateOffset, 13);

_weightedStateError.segment(stateOffset, 13).noalias() =
    stateWeight * _stateError.segment(stateOffset, 13);
```

권장 변경:

```cpp
const Mat13d T_k = bodyYawErrorTransform(referenceTrajectory.psi[k]);
const StateWeightMat Q_world_k = T_k.transpose() * stateWeight * T_k;

_weightedB.middleRows(stateOffset, 13).noalias() =
    Q_world_k * B_qp.middleRows(stateOffset, 13);

_weightedStateError.segment(stateOffset, 13).noalias() =
    Q_world_k * _stateError.segment(stateOffset, 13);
```

이렇게 하면 objective는 다음과 같이 바뀐다.

```text
(B_k u + A_k x0 - x_ref_k)^T
  * T_k^T * stateWeight * T_k
  * (B_k u + A_k x0 - x_ref_k)
```

`T_k` 예시:

```text
T_k = Identity(13)

T_k[0:2, 0:2] or T_k[0:3, 0:3]:
  roll/pitch error를 body-yaw frame 의미로 회전할지 결정

T_k[3:6, 3:6]:
  Rz(psi_ref_k)^T

T_k[6:9, 6:9]:
  Rz(psi_ref_k)^T

T_k[9:12, 9:12]:
  Rz(psi_ref_k)^T

T_k[12, 12]:
  1
```

처음 구현은 position, velocity, angular velocity block만 회전시키는 것이 안전하다.
roll/pitch block은 Euler angle convention 때문에 별도 검증 후 넣는 편이 좋다.

추천 1차 구현:

```text
rotate:
  position [px, py, pz]
  angular velocity [omega_x, omega_y, omega_z]
  linear velocity [vx, vy, vz]

keep as-is:
  roll
  pitch
  yaw
  gravity
```

추천 2차 구현:

```text
also rotate:
  roll/pitch tilt error using yaw-only small-angle approximation
```

## 6. Contact Wrench Constraint도 확인할 것

회전 중 넘어짐이 cost frame만의 문제는 아닐 수 있다.
현재 `GaitScheduler::buildConstraintMatrices()`의 friction/wrench constraint는 `Fx, Fy, Fz, Mx, My, Mz`에 직접 적용된다.

특히 다음 foot geometry 항은 foot frame 또는 foot-yaw frame 기준이어야 자연스럽다.

```text
foot_half_length
foot_half_width
torsional_friction_scale
```

만약 `M_x`, `M_y` constraint가 world axis 기준으로 고정돼 있으면,
발이 yaw 회전했을 때 foot length/width 방향이 world x/y와 맞지 않는다.

따라서 cost frame 수정 후에도 회전 중 이상하면 다음을 확인한다.

- full wrench cone이 foot local frame 기준으로 적용되는지
- `foot_half_length`가 foot forward axis와 연결되는지
- `foot_half_width`가 foot lateral axis와 연결되는지
- `NoRollMoment` 모드처럼 foot local x-axis를 쓰는 로직을 full wrench constraint에도 확장해야 하는지

권장 방향은 force/moment decision variable은 world로 유지하되,
constraint 평가 시에만 foot contact yaw frame으로 회전하는 것이다.

```text
F_F = R_FW * F_W
M_F = R_FW * M_W
C_foot * [F_F, M_F] <= bound

=> C_foot * blockdiag(R_FW, R_FW) * [F_W, M_W] <= bound
```

## 7. 빠른 검증 순서

구현 전에 frame 문제가 맞는지 빠르게 확인하려면 weight를 isotropic하게 맞춰 본다.

1. `px == py`로 둔다.
2. `vx == vy`로 둔다.
3. 가능하면 `roll == pitch`, `omega_x == omega_y`도 맞춘다.
4. yaw 회전 또는 curve walking에서 안정성이 개선되는지 본다.

이 테스트에서 안정성이 크게 좋아지면, cost weight가 world axis에 고정된 것이 주요 원인일 가능성이 높다.

그 다음 구현 순서는 다음이 좋다.

1. `ConvexMPC`에 step별 body-yaw error transform을 추가한다.
2. position, velocity, angular velocity error에만 먼저 적용한다.
3. 기존 isotropic test를 원래 anisotropic weight로 되돌려 yaw 회전 안정성을 비교한다.
4. 필요하면 roll/pitch error transform을 추가한다.
5. 그래도 회전 중 발 wrench가 이상하면 contact wrench constraint를 foot frame 기준으로 바꾼다.

## 8. 정리

권장 결론:

- desired body target은 world position/yaw로 유지한다.
- foot target도 최종 output은 world position으로 유지한다.
- user command와 foot offset은 body-yaw frame으로 유지한다.
- MPC dynamics는 world frame으로 유지한다.
- MPC state cost만 reference yaw 기준 body-yaw frame에서 평가한다.
- contact wrench constraint는 cost 수정 후에도 문제가 남으면 foot frame 기준으로 재검토한다.

이 접근이 전체 formulation을 body frame으로 바꾸는 것보다 변경 범위가 작고,
현재 코드 구조와도 가장 잘 맞는다.

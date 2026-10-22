"""Illustrative PID sensitivity study; no X35S model or hardware writes.

All quantities, including time, are normalized. Gains are NOT driver parameters.
Run from the repository root; outputs go to the ignored build directory.
"""

import csv
import math
import os
from pathlib import Path


def simulate(kp, ki, kd, output_limit=2.0, integral_limit=2.0,
             motor_gain=1.0, dt=0.001):
    position = velocity = integral = 0.0
    samples = []
    for step in range(round(30.0 / dt) + 1):
        time = step * dt
        error = -position
        raw_output = kp * error + integral - kd * velocity
        output = max(-output_limit, min(output_limit, raw_output))
        samples.append((time, error, output, integral, raw_output))
        if step == round(30.0 / dt):
            break
        if step == round(1.0 / dt):
            velocity += 0.6  # Assumed impact, not measured on the robot.
        integral = max(-integral_limit, min(integral_limit, integral + ki * error * dt))
        raw_output = kp * error + integral - kd * velocity
        output = max(-output_limit, min(output_limit, raw_output))
        load = 0.5 if time >= 1.0 else 0.0
        force = motor_gain * output - load - 1.2 * velocity
        # Unit inertia; assumed static/kinetic friction of 0.08/0.06.
        if abs(velocity) < 0.00001 and abs(force) <= 0.08:
            velocity = 0.0
        else:
            direction = velocity if abs(velocity) >= 0.00001 else force
            next_velocity = velocity + (force - 0.06 * math.copysign(1.0, direction)) * dt
            if velocity * next_velocity < 0.0 and abs(motor_gain * output - load) <= 0.08:
                next_velocity = 0.0
            velocity = next_velocity
        position += velocity * dt
    return samples


def main():
    cases = {
        "P only": dict(kp=4.0, ki=0.0, kd=3.0),
        "Double Kp": dict(kp=8.0, ki=0.0, kd=3.0),
        "Weak Ki": dict(kp=4.0, ki=0.1, kd=3.0),
        "Effective Ki": dict(kp=4.0, ki=2.0, kd=3.0),
        "Output limited": dict(kp=4.0, ki=2.0, kd=3.0, output_limit=0.5),
        "Limited, higher gains": dict(kp=8.0, ki=4.0, kd=3.0, output_limit=0.5),
        "Integral limited": dict(kp=4.0, ki=2.0, kd=3.0, integral_limit=0.03),
    }
    traces = {name: simulate(**parameters) for name, parameters in cases.items()}
    residual = {name: abs(trace[-1][1]) for name, trace in traces.items()}
    if not (residual["Double Kp"] < residual["P only"] * 0.6
            and residual["Effective Ki"] < 0.01 < residual["Weak Ki"]
            and residual["Output limited"] > 0.05
            and residual["Limited, higher gains"] > 0.05
            and residual["Integral limited"] > 0.05):
        raise RuntimeError("Sensitivity checks failed")

    # With unknown actuator gain, different numeric PID gains can give the
    # exact same position history: multiply gains/limits by c, motor gain by 1/c.
    scaled = simulate(kp=40.0, ki=20.0, kd=30.0, output_limit=20.0,
                      integral_limit=20.0, motor_gain=0.1)
    scale_error = max(abs(a[1] - b[1]) for a, b in zip(traces["Effective Ki"], scaled))
    if scale_error > 1e-10:
        raise RuntimeError("Actuator/gain scale equivalence check failed")
    convergence_error = max(abs(traces[name][-1][1] - simulate(**parameters, dt=0.0005)[-1][1])
                            for name, parameters in cases.items())
    if convergence_error > 0.002:
        raise RuntimeError("Time-step convergence check failed")

    destination = Path(__file__).resolve().parents[1] / "build" / "axis6-pid-analysis"
    destination.mkdir(parents=True, exist_ok=True)
    with (destination / "traces.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("case", "time_normalized", "error_normalized", "output_normalized",
                         "integral_output_normalized", "raw_output_normalized"))
        for name, trace in traces.items():
            writer.writerows((name, *sample) for sample in trace[::10])
    lines = ["ILLUSTRATIVE NORMALIZED MODEL; NOT X35S GAINS", "",
             "case | error at t=4 | error at t=30 | final output | final integral"]
    for name, trace in traces.items():
        lines.append(f"{name} | {trace[4000][1]:.6f} | {trace[-1][1]:.6f} | "
                     f"{trace[-1][2]:.6f} | {trace[-1][3]:.6f}")
    lines += ["", f"Scaled gain equivalence max error: {scale_error:.3g}",
              f"Half-step final error difference: {convergence_error:.6g}",
              "Self-checks: PASS. No real motor parameters identified or written."]
    (destination / "results.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

    os.environ["MPLCONFIGDIR"] = str(destination / "matplotlib")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    for name, trace in traces.items():
        for axis in axes:
            axis.plot([sample[0] for sample in trace[::10]],
                      [sample[1] for sample in trace[::10]], label=name, linewidth=1.5)
    for axis, end in zip(axes, (5, 30)):
        axis.set(xlim=(0, end), xlabel="Normalized time", ylabel="Normalized position error")
        axis.axhline(0, color="black", linewidth=0.6)
        axis.grid(alpha=0.2)
    axes[0].set_title("Impact and early recovery")
    axes[1].set_title("Residual error and controller limits")
    axes[1].legend(fontsize=8, loc="lower right")
    figure.suptitle("Illustrative sensitivity study: NOT an X35S model or tuning result")
    figure.savefig(destination / "sensitivity.png", dpi=160)
    plt.close(figure)
    print("\n".join(lines))
    print(f"Artifacts: {destination}")


if __name__ == "__main__":
    main()

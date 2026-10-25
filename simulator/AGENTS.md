# Development

- `dist/` contains editable HTML, CSS and JavaScript modules. Dependencies are vendored in `dist/vendor/`; preserve their license files.
- `server.mjs` serves the simulator locally. No bundler or package installation is required.
- Keep `dist/index.html` and `dist/shot.html` synchronized.
- Preserve robot controls, four-ball capacity, queue-head preview, front-only intake, side/rear pushing, flower release and gamepad support.
- Preserve demand-driven rendering and the fixed robot-mounted shooter.
- Run relevant checks after changes. `npm run check` runs all checks in `checks/`.

- Autonomous state, navigation, aiming and timing belong in `robot/shared/` Java only. Browser modules provide physics, sensors, simulated actuators and transport; do not duplicate autonomous decisions in JavaScript.
- Keep the shared Java sources compatible with Java 8 and free of FTC SDK/desktop imports. `robot/ftc/` is the SDK adapter; `robot/desktop/` and `bridge/` are local-only.
- Sensor timestamps use the same clock as the controller. Never infer ball counts from pickup/feed timers. Use measured inventory and preserve stop-on-disconnect/manual override.

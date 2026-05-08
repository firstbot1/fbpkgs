# Code Style & Naming (vio_bridge)

This project follows a pragmatic subset of widely adopted C++/ROS 2 conventions:

- Constants: `kCamelCase` with `k` prefix (e.g., `kParamHistory`, `kDefaultDepth`).
- Parameters: centralize parameter keys/defaults in `include/vio_bridge/constants.h`.
- Functions/Methods: `lower_snake_case` for free functions, `CamelCase` for types.
- Namespaces: `vio_bridge::<area>` (e.g., `vio_bridge::qos`, `vio_bridge::utils`).
- Headers: one responsibility per file; keep headers minimal and include only what they use.
- Doxygen: add brief `@file` headers and API comments on public functions/classes.

Commenting guidelines (Doxygen):
- Use `@brief` one‑liners and short parameter docs.
- Document error cases and exceptions (e.g., `@throws std::invalid_argument`).
- Keep private/internal code self‑documenting; avoid redundant comments.

Where it maps to upstream guidance:
- Google C++ Style Guide for constant naming and general C++ idioms.
- ROS 2 parameters and package layout conventions.


# Agents

## Code Style

C# code style is enforced via the `.editorconfig` file in the repository root (`/.editorconfig`).

Key conventions:
- **Indentation**: Tabs (width 4)
- **Braces**: Allman style (opening brace on its own line)
- **Naming**:
  - `PascalCase` for public members, types, methods, properties, constants, and enums
  - `_camelCase` (underscore prefix) for private and protected fields
  - `camelCase` for local variables and parameters
  - `I` prefix for interfaces (e.g., `IRespawnablePlayer`)
  - `T` prefix for type parameters
- **`using` directives**: Outside namespace, System directives first
- **`var`**: Allowed when the type is apparent from the right-hand side; prefer explicit types otherwise
- **Expression-bodied members**: Allowed for single-line properties, accessors, and lambdas
- **Braces**: Always required around control flow blocks
- **Namespaces**: Block-scoped (not file-scoped)

IDEs that support `.editorconfig` (Visual Studio, VS Code with C# extension, Rider) will automatically pick up these rules.

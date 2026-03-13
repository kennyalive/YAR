# YAR Agent Notes

Project preferences for AI-assisted code changes:

- Avoid introducing small helper functions too early. Inline one-off logic first. Extract helpers when functionality is clearly shared or repeated.
- Do not introduce metaprogramming unless it has a clear, concrete benefit.
- Do not optimize for theoretical performance.
- Keep changes straightforward and easy to read.

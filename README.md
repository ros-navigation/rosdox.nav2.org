# api.nav2.org

Hosting Nav2 & ROS 2 doxygen in a developer-friendly format.

Totally vibe-coded with Claude Code to the minimum degree required to function.
Read the source code and submit PRs at your own (sanity's) risk.
Contributions welcome, but manual testing is required.

## Updates Note

Make sure new messages are asked to have the result enums be in the description for ``error_codes``.

When updating with new action definitions, ask Claude to generate useage examples and extract the data from nav2_msgs to populate the descriptions. Look over the descriptions and tell it what to update them with if not sufficient. Make sure they're placed into the correct section of the Actions API page.

# api.nav2.org

Hosting Nav2 & ROS 2 doxygen in a developer-friendly format.

Totally vibe-coded with Claude Code to the minimum degree required to function.
Read the source code and submit PRs at your own (sanity's) risk.
Contributions welcome, but manual testing is required.

## Updates Note

When updating with new action, services, messages definition(s), ask Claude to generate useage examples with the full API and extract the data from nav2_msgs to populate the descriptions table. Look over the descriptions and tell it what to update them with if not sufficient. Make sure they're placed into the correct section of the Actions API page & alphabetized. Make sure new messages are asked to have the result enums be in the description for ``error_codes``.

When updating actions with new fields, ask it to download and check all actions for new fields. If any found, put them in the description tables with a detailed description of what they do and an example usage in the C++ and Python examples.

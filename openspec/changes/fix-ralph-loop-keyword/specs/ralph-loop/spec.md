## ADDED Requirements

### Requirement: Keyword activation triggers on actual message events
The ralph-loop plugin SHALL activate when a user message contains the configured keyword, by handling both message.updated and message.part.updated events with de-duplication.

#### Scenario: Keyword present in user message
- **WHEN** a user message event (message.updated or message.part.updated) contains the keyword (e.g., "ralph")
- **THEN** the plugin activates for the session and injects the completion instruction

#### Scenario: Keyword absent in user message
- **WHEN** a user message event does not contain the keyword
- **THEN** the plugin does not activate and does not inject the completion instruction

### Requirement: Activation persists for the session loop
Once activated, the ralph-loop plugin SHALL continue its loop behavior for that session until completion criteria are met.

#### Scenario: Active session continues loop
- **WHEN** the session is active and idle without the promise word
- **THEN** the plugin performs summarize → save → restart flow

# Tasks: Tasks Plugin Unification & Session Isolation

## Task List

### Phase 1: Core Implementation (Week 1)

#### P1: Unified Tool Foundation
- [ ] **1.1** Create unified command handler (`src/commands/unified.ts`)
  - [ ] Implement `unifiedCommand()` function
  - [ ] Add operation routing logic
  - [ ] Implement partial failure handling
  - [ ] Add comprehensive error handling

- [ ] **1.2** Implement session isolation in storage (`src/lib/storage.ts`)
  - [ ] Add session-aware read/write methods
  - [ ] Create session directory structure
  - [ ] Remove cross-session listing functionality
  - [ ] Add session validation

- [ ] **1.3** Create unified output formatter (`src/lib/formatter.ts`)
  - [ ] Implement `formatSessionStatus()` function
  - [ ] Create session-specific markdown formatter
  - [ ] Add progress bar and emoji support
  - [ ] Ensure consistent output format

- [ ] **1.4** Define unified types (`src/types/index.ts`)
  - [ ] Add `UnifiedOperation` type
  - [ ] Add `UnifiedCommandParams` type
  - [ ] Add `UnifiedCommandResult` type
  - [ ] Update existing types for compatibility

#### P2: Tool Integration
- [ ] **2.1** Implement unified `tasks` tool in `src/index.ts`
  - [ ] Define tool schema with operations array
  - [ ] Implement execute function
  - [ ] Add session ID extraction from context
  - [ ] Integrate with unified command handler

- [ ] **2.2** Refactor existing commands for reuse
  - [ ] Extract core logic from `init.ts`
  - [ ] Extract core logic from `add.ts`
  - [ ] Extract core logic from `update.ts`
  - [ ] Extract core logic from `complete.ts`
  - [ ] Extract core logic from `remove.ts`

- [ ] **2.3** Update operation handlers
  - [ ] Ensure all handlers accept sessionId parameter
  - [ ] Update storage calls to use session isolation
  - [ ] Add input validation for each operation
  - [ ] Add operation-specific error messages

### Phase 2: Documentation (Week 2)

#### P3: Migration Guide
- [ ] **3.1** Write migration guide
  - [ ] Document before/after syntax
  - [ ] Provide common use case examples
  - [ ] Add troubleshooting section

- [ ] **3.2** Update existing documentation
  - [ ] Update README.md
  - [ ] Update AGENTS.md references
  - [ ] Update tool documentation
  - [ ] Remove old tool references

### Phase 3: Testing & Validation (Week 3)

#### P5: Unit Testing
- [ ] **5.1** Test unified command handler
  - [ ] Test single operation execution
  - [ ] Test batch operation execution
  - [ ] Test partial failure handling
  - [ ] Test error propagation

- [ ] **5.2** Test session isolation
  - [ ] Test cross-session data isolation
  - [ ] Test session directory creation
  - [ ] Test session-specific reads/writes
  - [ ] Test default session fallback

- [ ] **5.3** Test operation types
  - [ ] Test 'init' operation
  - [ ] Test 'add' operation (with/without parent)
  - [ ] Test 'update' operation
  - [ ] Test 'complete' operation
  - [ ] Test 'remove' operation

#### P6: Integration Testing
- [ ] **6.1** End-to-end workflow tests
  - [ ] Test full task lifecycle
  - [ ] Test batch workflow
  - [ ] Test error recovery
  - [ ] Test output formatting

- [ ] **6.2** Multi-session tests
  - [ ] Test session A cannot see session B
  - [ ] Test concurrent session operations
  - [ ] Test session cleanup

- [ ] **6.3** Migration compatibility tests
  - [ ] Test deprecated tools still work
  - [ ] Test deprecation warnings display
  - [ ] Test mixed usage (old + new)

### Phase 4: Deployment & Cleanup (Week 4)

#### P7: Release Preparation
- [ ] **7.1** Version bump and changelog
  - [ ] Update package.json version
  - [ ] Write CHANGELOG.md entry
  - [ ] Tag release

- [ ] **7.2** Final validation
  - [ ] Run full test suite
  - [ ] Verify build success
  - [ ] Check code coverage
  - [ ] Performance benchmark

#### P8: Cleanup (Completed in Phase 1)
- [x] **8.1** Old tools removed (hard deprecation)
  - [x] Deleted old tool definitions
  - [x] Removed old command files
  - [x] Cleaned up old utilities

## Acceptance Criteria

### Functional Requirements
- [ ] `tasks` tool supports all 5 operation types (init, add, update, complete, remove)
- [ ] `tasks` tool handles up to 50 operations in one call
- [ ] `tasks` tool shows current session status after every execution
- [ ] Session isolation prevents cross-session data access
- [ ] Partial failure is supported (failed operations don't rollback successful ones)
- [ ] All existing functionality is preserved through unified interface

### Non-Functional Requirements
- [ ] Response time < 100ms for single operation
- [ ] Response time < 500ms for 50 batch operations
- [ ] 100% backward compatibility during deprecation period
- [ ] Code coverage > 90%
- [ ] All tests passing

### User Experience Requirements
- [ ] Clear deprecation warnings on old tools
- [ ] Comprehensive migration guide available
- [ ] Consistent output format across all operations
- [ ] Helpful error messages for invalid operations
- [ ] Automatic session management (no user input required)

## Dependencies

### Internal Dependencies
- `src/types/index.ts` - Type definitions
- `src/lib/storage.ts` - Storage layer
- `src/lib/formatter.ts` - Output formatting
- `src/lib/parser.ts` - Task parsing

### External Dependencies
- OpenCode Plugin API
- File system utilities (fs/promises)
- Path utilities (path)

## Risk Mitigation

| Risk | Mitigation Strategy |
|------|---------------------|
| Breaking existing workflows | 2-month deprecation period with clear warnings |
| Data loss during migration | Comprehensive backup strategy + rollback plan |
| Session isolation bugs | Extensive multi-session testing |
| Performance degradation | Benchmarking + optimization |
| User confusion | Detailed documentation + examples |

## Notes

- **Priority**: High (affects all agent workflows)
- **Estimated Effort**: 4 weeks
- **Breaking Change**: Yes (with deprecation period)
- **Rollback Plan**: Restore from git + clear plugin cache

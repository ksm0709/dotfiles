# Session Todo Manager Tasks

## Implementation Tasks

### Phase 1: TypeScript Wrapper Development
- [ ] Create `opencode/config/tools/session-todo.ts`
- [ ] Implement OpenCode Custom Tool interface
- [ ] Add session ID extraction from context
- [ ] Create Python subprocess integration
- [ ] Add argument validation and error handling

### Phase 2: Python Core Enhancement
- [ ] Move `simple-todo.py` to `opencode/config/tools/`
- [ ] Add CLI argument parsing interface
- [ ] Implement project-relative path resolution
- [ ] Add session-aware directory management
- [ ] Maintain backward compatibility

### Phase 3: Path Management System
- [ ] Create project root detection utility
- [ ] Implement session base path resolution
- [ ] Add directory creation with proper permissions
- [ ] Create session metadata management
- [ ] Add path validation and security checks

### Phase 4: Integration and Testing
- [ ] Test TypeScript wrapper session ID extraction
- [ ] Verify Python core CLI interface functionality
- [ ] Test session persistence across multiple calls
- [ ] Validate project-relative path resolution
- [ ] Test backward compatibility with existing usage

### Phase 5: Documentation and Deployment
- [ ] Update subagent usage documentation
- [ ] Create integration guide for developers
- [ ] Add troubleshooting documentation
- [ ] Update AGENTS.md with new tool locations
- [ ] Create migration guide for existing implementations

## Validation Criteria

### Functional Requirements
- [ ] Session ID persists across multiple subagent calls
- [ ] TypeScript wrapper successfully extracts session ID from OpenCode context
- [ ] Python core maintains all existing functionality
- [ ] Project-relative paths work without hardcoded dependencies
- [ ] Backward compatibility preserved for direct Python usage

### Non-Functional Requirements
- [ ] No hardcoded absolute paths in implementation
- [ ] Proper error handling for missing session IDs
- [ ] Secure path resolution preventing traversal attacks
- [ ] Performance acceptable for subagent usage patterns
- [ ] Clean integration with OpenCode tool ecosystem

### Integration Requirements
- [ ] Tool works as OpenCode Custom Tool
- [ ] Subagents can use tool without modification
- [ ] Existing Python scripts continue to work
- [ ] Session data properly isolated between agents
- [ ] Tool discovery and help functionality working

## Dependencies

### External Dependencies
- [ ] OpenCode SDK for TypeScript tool interface
- [ ] Node.js runtime for TypeScript execution
- [ ] Python 3.x runtime for core functionality

### Internal Dependencies
- [ ] Existing SimpleTodoManager codebase
- [ ] OpenCode project structure and conventions
- [ ] Subagent integration patterns
- [ ] Project configuration management

## Risk Mitigation

### Technical Risks
- [ ] OpenCode Context API availability verified
- [ ] TypeScript-Python integration tested thoroughly
- [ ] Path resolution works across different project structures
- [ ] Performance impact on subagent operations measured

### Compatibility Risks
- [ ] Existing subagent usage tested without modification
- [ ] Direct Python usage patterns preserved
- [ ] Tool migration path documented and tested
- [ ] Rollback strategy prepared if issues arise

## Success Metrics

### Quantitative Metrics
- [ ] Session ID persistence success rate: 100%
- [ ] Tool response time: < 200ms for typical operations
- [ ] Backward compatibility: 100% of existing usage patterns work
- [ ] Path resolution success rate: > 95% across project structures

### Qualitative Metrics
- [ ] Subagent developer experience improved
- [ ] Code maintainability enhanced through proper structure
- [ ] Integration with OpenCode ecosystem seamless
- [ ] Documentation comprehensive and clear
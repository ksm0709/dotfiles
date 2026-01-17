# Tasks: Transition Vibe-Kanban to YAML Configuration

- [ ] Prepare YAML template <!-- id: 0 -->
    - [ ] Rename `vibe-kanban/config-template.sh` to `vibe-kanban/config-template.yaml` <!-- id: 1 -->
    - [ ] Update content of `config-template.yaml` to YAML format <!-- id: 2 -->
- [ ] Update installation logic <!-- id: 3 -->
    - [ ] Modify `vibe-kanban/install.sh` to use `.yaml` instead of `.sh` <!-- id: 4 -->
    - [ ] Update `vibe-kanban/install-service.sh` to remove `EnvironmentFile` dependency <!-- id: 5 -->
- [ ] Update execution scripts <!-- id: 6 -->
    - [ ] Implement YAML parsing in `vibe-kanban/vibe-kanban-wrapper.sh` <!-- id: 7 -->
    - [ ] Implement YAML parsing in `vibe-kanban/vk-runner.sh` <!-- id: 8 -->
- [ ] Verification <!-- id: 9 -->
    - [ ] Run `install.sh` and verify `.yaml` creation <!-- id: 10 -->
    - [ ] Verify manual execution loads YAML config <!-- id: 11 -->
    - [ ] Verify systemd service loads YAML config <!-- id: 12 -->

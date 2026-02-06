#!/bin/bash

opencode upgrade
openchamber update
sudo systemctl restart openchamber.service

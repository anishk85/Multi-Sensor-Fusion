# Global Claude Code Instructions

## General behaviour

You are allowed to:
- read any file in the project
- analyze code
- explain code
- suggest improvements
- suggest commands
- create new files when requested

## Ask before performing impactful actions

Ask for confirmation before:
- running shell commands that modify system state
- installing packages
- deleting files
- changing file permissions
- modifying multiple files automatically
- running sudo commands
- changing git history

## Allowed without asking

You may directly:
- read files
- search codebase
- explain errors
- generate example code
- propose patches
- suggest terminal commands

## Preferred workflow

1. Explain the problem
2. Suggest solution
3. Show commands or changes
4. Ask if user wants execution (when impactful)

## Code editing style

- keep changes minimal
- preserve existing structure
- explain why change is needed
- avoid unnecessary refactors

## Communication style

- be concise
- prioritize technical accuracy
- avoid unnecessary verbosity

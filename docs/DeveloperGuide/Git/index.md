# Git 

AWSIM is managed by [git](https://git-scm.com/) and [GitHub](https://github.com/).

## Policy

- All git commit history is not modified
- Disallow `git rebase`
- Disallow `git push --force`
- Fast forward recommended
- Include `.meta` files

## Branch

```mermaid
---
config:
  logLevel: 'debug'
  theme: 'default'
  themeVariables:
      'git0': '#000000'
      'git1': '#000000'
      'git2': '#000000'
      'git3': '#000000'
      'gitBranchLabel0': '#ffffff'
      'gitBranchLabel1': '#ffffff'
      'gitBranchLabel2': '#ffffff'
      'gitBranchLabel3': '#ffffff'
---
  gitGraph
    checkout main
    commit
    branch feature/xxx
    commit
    checkout main
    merge feature/xxx
    checkout main
    branch release/xxx
    checkout release/xxx
    commit
    checkout main
    merge release/xxx
    checkout main
    commit

```

|Branch|Explain|
|:--|:--|
|main|Mainstream branch. Latest each AWSIM features and Lexus demo binary scene are included.|
|feature/*|Feature development branch created from the `main` branch.|
|release/*|Release branch of AWSIM. New AWSIM Lexus demo binary is created for each release.|
|gh-pages|Documantation hosted on GitHub pages (This web site).  When committed to the main branch, the documentation will be deployed by [GitHub-Actions](https://github.com/tier4/AWSIM/blob/main/.github/workflows/documentation_generation.yml).|
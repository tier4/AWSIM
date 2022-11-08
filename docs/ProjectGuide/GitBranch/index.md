# Git branch

The document presents the rules of branching adopted in the AWSIM development process.

## Branches

|branch|explain|
|:--|:--|
|main|Stable branch. Contains all the latest releases.|
|feature/***|Feature implementation branch created from `main`. <br>After implementation, it is merged into `main`.|
|gh-pages|Documentation hosted on GitHub pages.|

## Branch flow

1. Create `feature/***` branch from `main`.
2. Implement in `feature/***` branch.
3. Create a PR from the `feature/***` branch to `main` branch. Merge after review.

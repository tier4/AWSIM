The following are the recommended operational practices when AWSIM is forked or copied for custom use.

!!! info 
    The architecture can be freely designed. However, if you are in doubt, this document may be helpful.

## Recommended rules
- Avoid making changes to existing AWSIM assets in order to merge updates from the original AWSIM into your custom AWSIM.
- If you want to make changes to the original AWSIM, please refer to the [How to contribute](../HowToContribute//index.md) documentation.
- Decide whether to use the HDRP or URP rendering pipeline. see also [Switch SRP](../SwitchSrp/index.md) documentation.
- Create a directory for the newly added assets and include the additional assets there. For example, this directory structure.
    ```{.yml .no-copy}
    Awsim       //  root directory.
     │
     │
     ├─Assets                     
     │  │                         
     │  │
     │  ├─Awsim                // Default AWSIM assets.
     │  │  │                   // Assets contained in this directory 
     │  │  │                   // avoid change.
     │  │  │
     │  │  │
     │  │  ├─Prefabs
     │  │  ├─Models
     │  │  ├─etc.
     │  │  ┊︎      
     │  │                        
     │  ├─<Custom dir>         // Additional AWSIM assets.
     │  │  │                   // Include additional assets in 
     │  │  │                   // any custom directory you create.
     │  │  │
     │  │  │               
     │  │  ├─Prefabs
     │  │  ├─Models
     │  │  ├─etc.
     │  │  ┊︎
     │  │
     │  │ 
     ┊︎  ┊︎  
    ```
    
## Updating from Tier IV AWSIM

Whether you **fork** or **copy** the AWSIM repository, the workflow for keeping your custom AWSIM up to date with the original Tier IV AWSIM is essentially the same.

- **Fork**:  
  Clicking the Fork button on GitHub creates a replica under your account (`origin`). However, when cloning locally, only `origin` is configured by default — no `upstream` remote is set automatically.  

- **Copy (private repository)**:  
  Creating a private repository and pushing the original AWSIM code into it results in the same situation: only `origin` is configured.  

In both cases, you must **manually add `upstream`** to pull updates from Tier IV AWSIM.

1. Set upstream
In your custom AWSIM repository, run:
```bash
git remote add upstream git@github.com:tier4/AWSIM.git
```
1. Fetch and merge updates

    Fetch the latest changes from the Tier IV repository:
    ```bash
    git fetch upstream
    ```

1. Merge or rebase them into your local main branch:
    ```bash
    git merge upstream/main
    ```


1. Push the updated branch to your own repository (fork or private copy):
    ```bash
    git push origin main
    ```
!!! info
    Notes on pull / push

    By default, git pull and git push only operate on origin (your fork or private repository).

    To retrieve updates from Tier IV AWSIM, you must explicitly specify upstream:
    ```bash
    git pull upstream main
    ```

    A normal push:
    ```bash
    git push
    ```

    will only update your repository (origin), never the Tier IV AWSIM repository.

!!! tip
    Fork and Copy behave the same when synchronizing with Tier IV AWSIM:   
    Add upstream → fetch → merge → push to your own repository.

## Namespace

It is a good idea to include it in AWSIM's existing namespace as appropriate. Please refer to the [Architecture](../Architecture/index.md) documentation.

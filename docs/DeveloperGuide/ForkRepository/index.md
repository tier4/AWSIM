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

## Namespace

It is a good idea to include it in AWSIM's existing namespace as appropriate. Please refer to the [Architecture](../Architecture/index.md) documentation.

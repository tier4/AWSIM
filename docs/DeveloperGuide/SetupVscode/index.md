## Install Visual Studio Code

Follow the steps in:
[https://code.visualstudio.com/docs/setup/linux](https://code.visualstudio.com/docs/setup/linux)

```bash
# Install the keys and repository
sudo apt-get install wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
rm -f packages.microsoft.gpg

# Then update the package cache and install the package using:
sudo apt install apt-transport-https
sudo apt update
sudo apt install code
```

## Install the Dotnet SDK

Follow the steps in:
[https://learn.microsoft.com/en-us/dotnet/core/install/linux-ubuntu#register-the-microsoft-package-repository](https://learn.microsoft.com/en-us/dotnet/core/install/linux-ubuntu#register-the-microsoft-package-repository)

```bash
# Get Ubuntu version
declare repo_version=$(if command -v lsb_release &> /dev/null; then lsb_release -r -s; else grep -oP '(?<=^VERSION_ID=).+' /etc/os-release | tr -d '"'; fi)

# Download Microsoft signing key and repository
wget https://packages.microsoft.com/config/ubuntu/$repo_version/packages-microsoft-prod.deb -O packages-microsoft-prod.deb

# Install Microsoft signing key and repository
sudo dpkg -i packages-microsoft-prod.deb

# Clean up
rm packages-microsoft-prod.deb

# Update packages
sudo apt update

sudo apt install dotnet-sdk-8.0
```

## Install the extensions

Follow the steps in:

- [https://marketplace.visualstudio.com/items?itemName=ms-dotnettools.csdevkit](https://marketplace.visualstudio.com/items?itemName=ms-dotnettools.csdevkit)
- [https://marketplace.visualstudio.com/items?itemName=VisualStudioToolsForUnity.vstuc](https://marketplace.visualstudio.com/items?itemName=VisualStudioToolsForUnity.vstuc)
- [https://marketplace.visualstudio.com/items?itemName=ms-dotnettools.csharp](https://marketplace.visualstudio.com/items?itemName=ms-dotnettools.csharp)

> Launch VS Code Quick Open (Ctrl+P), paste the following command, and press enter.  
  - `ext install ms-dotnettools.csdevkit`    
  Repeat for:  
  - `ext install VisualStudioToolsForUnity.vstuc`  
  - `ext install ms-dotnettools.csharp`  

## Configure the Unity

- Open up the Unity Editor
- `Edit` -> `Preferences` -> `External Tools` -> `External Script Editor`
- Select `Visual Studio Code`
  - If it's not there, click `Browse` and navigate and select `/usr/bin/code`

It should all be configured now.
You can either open up a script by double clicking in the Project window in Unity or by opening up the project in VS Code:
- `Assets` -> `Open C# Project`

Syntax highlighting and CTRL-click navigation should work out of the box.

For more advanced features such as debugging, check the [Unity Development with VS Code Documentation](https://code.visualstudio.com/docs/other/unity#_editing-evolved).

## Additional notes

In the AWSIM project, the package [Visual Studio Editor](https://docs.unity3d.com/2021.3/Documentation/Manual/com.unity.ide.visualstudio.html) is already installed to satisfy the requirement from the [Unity for Visual Studio Code](https://code.visualstudio.com/docs/other/unity#_update-the-visual-studio-package) extension.
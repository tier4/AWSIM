
## Headless

By default, AWSIM is a standard windowed application. This is core functionality and essential for running the simulation manually on a local PC with an attached display, but it starts to be problematic for CI and testing on the cloud instances.

Unity provides a few options for running binaries headless, but all of them have two things in common: 

- they still require a window server (X11 on Linux), or
- they disable the support for GPU devices.

Since AWSIM requires GPU for sensor simulation, it is required to use 3rd party utilities. The best utility for that is **Xvfb**.

!!! warning "Headless with Xvfb is only supported on Ubuntu"

## Xvfb

Xvfb, which stands for "X Virtual Framebuffer", is a display server implementing the X11 display server protocol. It enables the running of graphical applications without the need for a physical display by creating a virtual frame buffer in memory. 

This tool is transparent for the AWSIM and the pipeline in which it is working and can be conveniently used to mimic the headless functionalities.

### Installing

Xvfb is a standard Ubuntu package and can be installed with `apt`:

```bash
sudo apt update
sudo apt install xvfb 
```

### Running the AWSIM with Xvfb

To run the AWSIM binary, all that is needed to do, is to prefix the command with `xvfb-run`:

```bash
xvfb-run ./AWSIM.x86_64
```

Please note that `xvfb-run` comes with multiple options, like choosing the virtual screen size or color palette. Please see the [manual](https://manpages.ubuntu.com/manpages/xenial/man1/xvfb-run.1.html) for all the options.
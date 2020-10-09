# What it is

o80_pam is a wrappers over [pam_interface](https://intelligent-soft-robots.github.io/code_documentation/pam_interface/docs/html/index.html) providing [o80](https://intelligent-soft-robots.github.io/code_documentation/o80/docs/html/index.html) functionalities.

This documentation assumes you are familiar with both *pam_interface* and *o80*.

# Installation

Follow the [general guidelines](https://github.com/intelligent-soft-robots/intelligent-soft-robots.github.io/wiki), using either the treep project "PAM" or the treep project "PAM_MUJOCO" (if you'd like to use o80_pam over a PAM robot simulated by Mujoco).

If using mujoco, you also need to copy a mujoco licence key (mjkey.txt) in the folder /opt/mujoco/ (create the folder is necessary).

# Usage

## Starting a o80 server

In terminal, assuming the workspace has been [sourced](https://github.com/intelligent-soft-robots/intelligent-soft-robots.github.io/wiki/05_Compiling-a-project) :

To get some documentation:

```bash
o80_pam -h
```

To start a server over a dummy robot (no real physics, no graphics, just for debugging purposes):

```bash
o80_pam
```

To start a server over a mujoco simulated robot:

```bash
o80_pam mujoco
```

To start a server over the real robot (on cent-os control desktop, **assuming you follow the procedure provided in the pam_interface [documentation](https://intelligent-soft-robots.github.io/code_documentation/pam_interface/docs/html/index.html)**)

```bash
o80_pam real
```

## Checking all is working

Once the o80 server started, you may check all is working fine by running in another terminal :

```bash
o80_pam_check
```

This executable will changes the pressure on each muscle one by one.

## Performing a series of swing motion

```bash
o80_pam_lengthcables
```

## Python user code

You may send commands to the robot and receiving observations from it via an o80 based API.
The [o80 API](https://intelligent-soft-robots.github.io/code_documentation/o80/docs/html/index.html) is fully supported.
To get an example of usage of this PAI, see the demos folder. For example, this [demo](https://github.com/intelligent-soft-robots/o80_pam/blob/master/demos/demo.py). 

## Mirroring real robot by mujoco simulated robot

Once a o80_pam server is started:

```bash
pam_mujoco_mirroring
```


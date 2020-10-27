<h1 align="center"> Robotics Project </h1> <br>
<h3 align="center"> Planning, Developing, Learning ROS </h3> <br>
<p align="center">
University Of Burgundy (VIBOT)
  <p align="center">
      <img src = "vibot.png" width=60>
  </p>
</p>

<h3 align="center">                       
Supervisors: <br>  
 Ralph SEULIN
</h3>
<h4 align="center">                       
Students: <br>  
 Mahmoud Badran
</h4>
<p align="center">
  <p align = "center">
     <img  src = "https://www.ros.org/news/2016/05/23/kinetic.png" width=400>
  </p>
</p>


<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
## Project Management (Technical Report)

- [Introduction](#introduction)
- [Analysis OF Studied Techniques](#Analysis-OF-Studied-Techniques)
- [ROS Packages](#ROS-Packages)
- [Work Plan](#Work-Plan)
- [Conclusion](#Conclusion)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

## Introduction
Robotics Operating System (ROS), is a middleware, low level framework, to write robotic softwares. It can be cosidered as an API to make the process of developing a robotic related projects more flexible, and simplified. There will be no need for an extensive knowledge of the hardware in which it saves much effort and time in the development phase. It includes many libraries and tools which connects and control the robot manipulators, handle the comunication between multiple devices in a a working space. <br>
ROS is supported by many operating systems like ubunto, windows. Ubunto is the more stable operating system working with ROS. However, for the development of this project we are using the construct web plaform, which is an online robotics working environmant. The platform uses ubunto as the main operating system with ROS kinetic and uses the simulation of turtlebot 3, where we will be able to apply some of the techniques we have have learned to apply navigation and localization with map construction.  

## Analysis OF Studied Techniques

Let's explain some important concepts that have been studied and will be important in developing the project : <br><br>
**Nodes**: One of the most important concepts in ROS. We can describe nodes as a program (python, c++) to enable ROS to make communication tasks. A node can subscribe or publish to a topic, which will be explained. <br><br>
**Topic**: The process of transmitting data between nodes. Some nodes are responsible for publishing some data to a specific topic where other nodes (subscribers) will be able to request these data ( messages ) from the topic.<br><br>
**Messages** : Data structure which describes the data that ROS nodes publish or recieve. the nodes communicate, send messages, receive messages through topics.<br> 
**Services** : another way to transmit the data between nodes. it is a synchronous operation where the robot has to stop processing while waiting for a service response.<br><br>

<p align="center">
  <p align = "center">
     <img  src = "XCndTVvlwmaG.png" width=200>
  </p>
</p>

## ROS Packages

Feel free to send us feedback on [Twitter](https://twitter.com/gitpointapp) or [file an issue](https://github.com/gitpoint/git-point/issues/new). Feature requests are always welcome. If you wish to contribute, please take a quick look at the [guidelines](./CONTRIBUTING.md)!

If there's anything you'd like to chat about, please feel free to join our [Gitter chat](https://gitter.im/git-point)!

## Work Plan

This project follows the [all-contributors](https://github.com/kentcdodds/all-contributors) specification and is brought to you by these [awesome contributors](./CONTRIBUTORS.md).

## Conclusion

- Follow the [React Native Guide](https://facebook.github.io/react-native/docs/getting-started.html) for getting started building a project with native code. **A Mac is required if you wish to develop for iOS.**
- Clone or download the repo
- `yarn` to install dependencies
- `yarn run link` to link react-native dependencies
- `yarn start:ios` to start the packager and run the app in the iOS simulator (`yarn start:ios:logger` will boot the application with [redux-logger](<https://github.com/evgenyrodionov/redux-logger>))
- `yarn start:android` to start the packager and run the app in the the Android device/emulator (`yarn start:android:logger` will boot the application with [redux-logger](https://github.com/evgenyrodionov/redux-logger))

Please take a look at the [contributing guidelines](./CONTRIBUTING.md) for a detailed process on how to build your application as well as troubleshooting information.

**Development Keys**: The `CLIENT_ID` and `CLIENT_SECRET` in `api/index.js` are for development purposes and do not represent the actual application keys. Feel free to use them or use a new set of keys by creating an [OAuth application](https://github.com/settings/applications/new) of your own. Set the "Authorization callback URL" to `gitpoint://welcome`.

## Backers [![Backers on Open Collective](https://opencollective.com/git-point/backers/badge.svg)](#backers)

Thank you to all our backers! 🙏 [[Become a backer](https://opencollective.com/git-point#backer)]

<a href="https://opencollective.com/git-point#backers" target="_blank"><img src="https://opencollective.com/git-point/backers.svg?width=890"></a>

## Sponsors [![Sponsors on Open Collective](https://opencollective.com/git-point/sponsors/badge.svg)](#sponsors)

Support this project by becoming a sponsor. Your logo will show up here with a link to your website. [[Become a sponsor](https://opencollective.com/git-point#sponsor)]

<a href="https://opencollective.com/git-point/sponsor/0/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/0/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/1/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/1/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/2/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/2/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/3/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/3/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/4/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/4/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/5/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/5/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/6/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/6/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/7/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/7/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/8/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/8/avatar.svg"></a>
<a href="https://opencollective.com/git-point/sponsor/9/website" target="_blank"><img src="https://opencollective.com/git-point/sponsor/9/avatar.svg"></a>

## Acknowledgments

Thanks to [JetBrains](https://www.jetbrains.com) for supporting us with a [free Open Source License](https://www.jetbrains.com/buy/opensource).

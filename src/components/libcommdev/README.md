
<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/othneildrew/Best-README-Template">
    <img src="docs/img/icon.svg" alt="Main-Icon" width="120" height="120">
  </a>

  <h3 align="center">LibCommDev</h3>

  <p align="center">
    Libcommdev is a C library for embedded systems that provides modules for peripheral communication, including PWM, SPI, I2C, and more, with a focus on modularity, portability, and ease of use.
    <br />
    <!--<a href="https://github.com/othneildrew/Best-README-Template"><strong>Explore the docs »</strong></a>
    <br />-->
    <br />
    <a href="https://github.com/othneildrew/Best-README-Template">View Demo</a>
    ·
    <a href="https://github.com/bcostantino/libcommdev/issues">Report Bug / Request Feature</a>
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <!--<li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>-->
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About The Project

<!--[![Product Name Screen Shot][product-screenshot]](https://example.com)-->

LibCommDev is a powerful tool designed to simplify communication between different parts of an embedded system. It provides a modular and portable solution for handling different types of communication protocols, such as SPI, I2C, UART, and more. By using this library, developers can focus on building their application logic, rather than dealing with the complexities of low-level communication protocols. The library is designed to be flexible and easily integrated into different types of projects, making it a valuable asset for any embedded developer.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



**Target Directory Structure**

```
mylib/
├── docs/
│   └── index.md
├── examples/
│   ├── example1/
│   │   ├── CMakeLists.txt
│   │   ├── main.c
│   │   └── ...
│   ├── example2/
│   │   ├── CMakeLists.txt
│   │   ├── main.c
│   │   └── ...
│   └── ...
├── include/
│   ├── mylib_module1.h
│   ├── mylib_module2.h
│   └── ...
├── src/
│   ├── module1/
│   │   ├── CMakeLists.txt
│   │   ├── module1.c
│   │   └── ...
│   ├── module2/
│   │   ├── CMakeLists.txt
│   │   ├── module2.c
│   │   └── ...
│   └── ...
├── test/
│   ├── CMakeLists.txt
│   ├── module1_test.c
│   ├── module2_test.c
│   └── ...
├── CMakeLists.txt
├── LICENSE
└── README.md
```

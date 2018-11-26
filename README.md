# Introduction {#intro}

The following picture shows the block diagram of the embedded master dude architecture.

![Embedded Master Dude block diagram](/images/architecture_ti_cortexm4.svg "Embedded Master Dude Architecture")

# SW architecture and structure {#sw-arch}

This section shall give a short introduction to the core software modules that are relevant within this project.

## Measurement Framework {#mfw}

TODO: Add some brief description here.

## ProtoBuf {#nano-pb}

This project makes use of the [Protocol Buffers] library. As the coding language within this repository is C, the corresponding library is [NanoPb], 
that is added as separate Git submodule to this repository under `/submodule/nanopb`.

[NanoPb]: https://github.com/nanopb/nanopb "NanoPb"
[Protocol Buffers]: https://developers.google.com/protocol-buffers/ "Protocol Buffers"


## Utilities {#utils}

TODO: Add some brief description here.

- Message framing
- CRC16
- Ringbuf
- ...

## Hardware Abstraction Layer {#hal_docu}

The API documentation of the Hardware Abstraction Layer (HAL) can be found in the corresponding [HAL](@ref hal) section.

Furthermore, the HAL implementation is tested using dedicated test applications that are stored in folder `/test`. Additionally, these test apps serve as reference/example implementations for the HAL modules. Within the doxygen documentation these are located in the section example.

# Changelog {#changelog}

No official release created yet. However, this section shall later on give the reader a brief idea of the major changes applied to each released version.

| Release        | Date       | Description                                                                |
| :-----------:  | :--------: | :------------------------------------------------------------------------- |
| 0.1.0-alpha1   | 24.03.2016 | First Alpha                                                                |
| 0.1.0          | 20.06.2016 | First Pre-release\                                                         |
|                |            | - Last version compatible with betCOM 1.x                                  |
| 0.2.0-alpha1   | 23.06.2016 | Alpha release\                                                             |
|                |            | - First version compatible with betCOM 2.0.1-alpha2                        |
| 0.3.0-alpha1   | 04.07.2016 | Alpha release\                                                             |
|                |            | - First version compatible with betCOM 2.2.0-alpha1                        |
| 0.4.0-alpha1   | 27.07.2016 | Alpha release\                                                             |
|                |            | - First version compatible with betCOM 2.3.1-alpha1                        |
| 0.4.0          | 27.07.2016 | Release\                                                                   |
|                |            | - First version compatible with betCOM 2.3.1 and GS 0.2.0-alpha1           |
| 0.5.0          | 27.07.2016 | Release\                                                                   |
|                |            | - First version compatible with betCOM 2.4.5 and GS 0.3.0                  |

# Documentation {#docu}

The API documentation of the repository is managed and maintained using [Doxygen].

The Doxygen configuration file (`doxyfile-embMasterDude`) can be found in `/doc` folder of the repository.
Doxygen generates a new folder `/doc/output` that holds the documentation output. This folder is ignored by Git as the documentation can and shall be generated upon request any time.
Furthermore, Doxygen generates a lot of files and many changes within these files to create the documentation, that is not worth versioning as it creates more
extra work than benefit.

[Doxygen]: http://www.stack.nl/~dimitri/doxygen/index.html "Doxygen"

## DMA channel assignement
![DMA channel assignment](http://wiki.syscop.de/images/b/b6/Flight_Controller_DMA_channel_assignment.svg)
<img source="http://wiki.syscop.de/images/b/b6/Flight_Controller_DMA_channel_assignment.svg">


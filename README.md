# MyCPU: Verilog Implementation of a CPU Using Tomasulo Algorithm
## Project Overview
This is a model of a CPU designed and implemented in Verilog HDL that employs the Tomasulo algorithm for instruction scheduling. This project aims to simulate in detail the advanced dynamic scheduling mechanism in a processor pipeline using hardware description language, thereby enhancing execution efficiency under conditions with data dependencies.

### Key Features
- Tomasulo Algorithm: This CPU design utilizes the Tomasulo algorithm to address pipeline stalls caused by various types of data hazards such as structural, data, and control hazards.

- Full Functionality: It includes primary pipeline stages like fetch, decode, execute, memory access, and write-back, and implements key components like register renaming, Reservation Station, and Reorder Buffer (ROB).

- Modular Design: Each functional module, including ALU, floating-point unit, branch prediction unit, etc., is independently designed for easy maintenance and scalability.

## Contribution Guidelines
Feedback and improvement suggestions are welcome in any form! If you encounter any bugs or have new feature requests, please submit them via GitHub issues.

## License
This project is open-sourced under the MIT License.

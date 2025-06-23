# PIPELINE-PROCESSOR-DESIGN

*COMPANY* : CODTECH IT SOLUTIONS

*NAME* : Chetana Isunuri

*INTERN ID* : CT04DF2543

*DOMAIN* : VLSI

*DURATION* : 4 WEEKS

*MENTOR* : VAISHALI

# Objective of the Task:
The objective of this task is to design, implement, and simulate a 4-stage pipelined processor capable of executing fundamental operations such as ADD, SUB, and LOAD. The processor was written in Verilog HDL and simulated using Xilinx Vivado, a popular FPGA design suite. This task aims to provide hands-on experience in designing digital processors, understanding instruction pipelines, and improving instruction throughput using pipelining — a fundamental concept in modern processor design.

# Description :
This pipelined processor design is based on a simplified RISC-like architecture and follows a classic 4-stage pipeline model comprising the following stages:

1. Instruction Fetch (IF)

2. Instruction Decode (ID)

3. Execution (EX)

4. Memory Access and Write Back (MEM/WB)

Each stage is implemented using a dedicated logic block, and pipeline registers are placed between each stage to store intermediate results and maintain data flow between clock cycles. The processor supports three basic instructions:

ADD: Adds two register operands and stores the result in a destination register.

SUB: Subtracts the second register from the first and stores the result.

LOAD: Loads data from memory using a base register and offset.

These instructions follow a fixed 16-bit instruction format:
[opcode (4 bits)][rd (4 bits)][rs1 (4 bits)][rs2/imm (4 bits)]

The architecture uses:

A 16-location instruction memory

A 16-register file (each 8-bit wide)

A 256-byte data memory

The processor operates on a clock signal, and the pipeline logic is synchronized with the clock to mimic the real-time hardware flow.

# Pipeline Stages Explained:
-> IF Stage (Instruction Fetch):
Fetches instruction from instr_mem using the Program Counter (pc). The fetched instruction is passed to the next stage using the IF_ID_instr pipeline register.

-> ID Stage (Instruction Decode):
Decodes the fetched instruction into opcode, destination register (rd), and source registers (rs1, rs2). Source operands are read from the register file.

-> EX Stage (Execution):
Performs arithmetic operations (ADD, SUB) or address calculation for the LOAD instruction. Uses combinational logic (ALU) to calculate the result.

-> MEM/WB Stage (Memory Access and Write-Back):
If the instruction is LOAD, it reads data from data memory using the calculated address. Finally, the result (either from ALU or memory) is written back into the register file.

# Tools and Technologies Used:
@Xilinx Vivado: Used for design entry, simulation, waveform analysis, and debugging.

@Verilog HDL: The hardware description language used to model the pipelined CPU at RTL (Register Transfer Level).

@GTKWave (optional): For viewing signal waveforms from the VCD files generated during simulation.

# Applications:
1. Educational Purpose: This kind of processor is ideal for teaching microarchitecture, pipelining, and processor design fundamentals.

2. SoC Design: Simple pipelined cores can be embedded in System-on-Chip (SoC) designs for tasks like monitoring, control, or lightweight computation.

3. FPGA Prototyping: The design is synthesizable and can be deployed on FPGAs for experimental pipelined processor demos.

4. Custom CPU Development: Helps as a base for building more complex CPUs with hazard detection, branching, and forwarding.

# Learning Outcomes:
@ Developed a clear understanding of pipelining and data path control.

@ Learned how to manage pipeline registers between stages.

@ Gained practical knowledge in simulating CPU operations cycle by cycle.

@ Understood the importance of instruction encoding, memory interfaces, and control signals.

# Conclusion:
This task provided valuable experience in building a fundamental pipelined CPU architecture. Despite being simplified, the design introduces key processor concepts like instruction timing, parallel execution, register file usage, and memory access. It lays a strong foundation for more advanced projects like 5-stage RISC processors, hazard management units, or pipeline forwarding logic. The use of Verilog and Vivado made it easier to visualize and verify each pipeline stage's behavior, enabling a practical understanding of digital processor design.

# schematic & outputwaveforms
![Image](https://github.com/user-attachments/assets/10eb2bc3-f6f0-4c32-8c1c-ce64c56bfd54)
![Image](https://github.com/user-attachments/assets/2631c44f-f997-408b-a1b1-daa907a81237)


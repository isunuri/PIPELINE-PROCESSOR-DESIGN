module instr_mem (input [3:0] pc, output reg [15:0] instr);
    reg [15:0] mem [0:15];         // Instruction memory: 16 locations of 16-bit instructions

    initial begin
        // Format: [opcode(4 bits)][rd(4 bits)][rs1(4 bits)][rs2/imm(4 bits)]
        mem[0] = 16'b0001_0001_0010_0011; // ADD R1 = R2 + R3
        mem[1] = 16'b0010_0010_0011_0100; // SUB R2 = R3 - R4
        mem[2] = 16'b0011_0011_0000_0010; // LOAD R3 = MEM[R0 + 2]
        mem[3] = 16'b0001_0001_0010_0011; // ADD again
    end

    always @(*) begin
        instr = mem[pc];           // Output instruction at current PC
    end
endmodule

module reg_file (
    input clk,
    input we,                     // Write enable
    input [3:0] rd,               // Destination register address
    input [3:0] rs1, rs2,         // Source register addresses
    input [7:0] wd,               // Write data
    output [7:0] r1, r2           // Output data from source registers
);
    reg [7:0] regs [0:15];        // 16 registers of 8-bit width

    assign r1 = regs[rs1];        // Read data from source 1
    assign r2 = regs[rs2];        // Read data from source 2

    always @(posedge clk) begin
        if (we)
            regs[rd] <= wd;       // Write data to destination register on positive clock edge
    end
endmodule

module data_mem (input [7:0] addr, output [7:0] data);
    reg [7:0] mem [0:255];        // 256 bytes of data memory

    assign data = mem[addr];      // Output the data at the address

    initial begin
        mem[2] = 8'hAA;           // Pre-load memory address 2 with 0xAA for LOAD testing
    end
endmodule

module pipeline_cpu (input clk);
    reg [3:0] pc = 0;                             // Program Counter (4-bit for 16 instructions)
    wire [15:0] instr;                            // Instruction fetched from memory
    instr_mem imem (.pc(pc), .instr(instr));     // Instruction memory module

    //----- IF/ID Pipeline Register -----
    reg [15:0] IF_ID_instr;                      // Holds fetched instruction for decode stage

    //----- ID Stage -----
    wire [3:0] opcode = IF_ID_instr[15:12];      // Extract opcode
    wire [3:0] rd     = IF_ID_instr[11:8];       // Destination register
    wire [3:0] rs1    = IF_ID_instr[7:4];        // Source 1 register
    wire [3:0] rs2    = IF_ID_instr[3:0];        // Source 2 or immediate value
    wire [7:0] rs1_data, rs2_data;               // Register file outputs

    //----- ID/EX Pipeline Register -----
    reg [3:0] ID_EX_opcode, ID_EX_rd;            // Opcode and destination register
    reg [7:0] ID_EX_r1, ID_EX_r2;                // Source register values

    //----- EX/MEM Pipeline Register -----
    reg [7:0] EX_MEM_result;                     // ALU output
    reg [3:0] EX_MEM_rd;                         // Destination register from EX stage
    reg EX_MEM_we;                               // Write enable forwarded

    //----- MEM/WB Pipeline Register -----
    reg [7:0] MEM_WB_data;                       // Final write-back data
    reg [3:0] MEM_WB_rd;                         // Destination register in WB stage
    reg MEM_WB_we;                               // Write enable

    //----- Register File Instantiation -----
   // Write to register only in WB stage
    reg_file rf ( .clk(clk),.we(MEM_WB_we),.rd(MEM_WB_rd),.rs1(rs1),.rs2(rs2),.wd(MEM_WB_data),.r1(rs1_data),.r2(rs2_data));

    //----- Execution Stage Logic -----
    reg [7:0] alu_out;                           // ALU output result
    reg is_load;                                 // Control flag for load

    always @(*) begin
        is_load = 0;
        case (ID_EX_opcode)
            4'b0001: alu_out = ID_EX_r1 + ID_EX_r2;  // ADD
            4'b0010: alu_out = ID_EX_r1 - ID_EX_r2;  // SUB
            4'b0011: begin
                alu_out = ID_EX_r1 + ID_EX_rd;       // LOAD address: r1 + offset
                is_load = 1;
            end
            default: alu_out = 0;
        endcase
    end

    //----- Data Memory Access -----
    wire [7:0] mem_out;
    data_mem dmem (.addr(alu_out), .data(mem_out));  // For LOAD, use ALU result as address

    //----- Pipeline Register Updates (at every clock edge) -----
    always @(posedge clk) begin
        pc <= pc + 1;                              // Increment PC to fetch next instruction

        // IF to ID
        IF_ID_instr <= instr;

        // ID to EX
        ID_EX_opcode <= opcode;
        ID_EX_rd     <= rd;
        ID_EX_r1     <= rs1_data;
        ID_EX_r2     <= rs2_data;

        // EX to MEM
        EX_MEM_result <= alu_out;
        EX_MEM_rd     <= ID_EX_rd;
        EX_MEM_we     <= 1;                        // Always writing result (could be optimized)

        // MEM to WB
        MEM_WB_data <= is_load ? mem_out : EX_MEM_result;  // Choose between memory or ALU result
        MEM_WB_rd   <= EX_MEM_rd;
        MEM_WB_we   <= EX_MEM_we;
    end
endmodule

module pipeline_tb;
    reg clk = 0;
    pipeline_cpu cpu (.clk(clk));                 // Instantiate CPU

    always #5 clk = ~clk;                         // Clock generator: toggle every 5 time units

    initial begin
        $dumpfile("pipeline.vcd");                // VCD file for waveform viewer
        $dumpvars(0, pipeline_tb);

        // Print key outputs to console
        $monitor("T=%0t | PC=%0d | Instr=%b | R1=%h | R2=%h | ALU=%h | WB=%h",
            $time, cpu.pc, cpu.IF_ID_instr,
            cpu.rs1_data, cpu.rs2_data,
            cpu.EX_MEM_result, cpu.MEM_WB_data);

        #100 $finish;                             // End simulation after 100 time units
    end
endmodule


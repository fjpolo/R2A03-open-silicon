// @fjpolo
// https://github.com/MiSTer-devel/NES_MiSTer
`default_nettype 

module top(
                input  wire       clk_i,      // clock
                input  wire       rst_n_i     // reset_n - low to reset
            );

    /**********************************************************/
    /*************              CPU             ***************/
    /**********************************************************/

    wire [15:0] cpu_addr;
    wire cpu_rnw;
    wire pause_cpu;
    wire nmi;
    wire mapper_irq;
    wire apu_irq;

    // IRQ only changes once per CPU ce and with our current
    // limited CPU model, NMI is only latched on the falling edge
    // of M2, which corresponds with CPU ce, so no latches needed.

    T65 cpu(
        .Mode   (0),
        .BCD_en (0),

        .Res_n  (~reset),
        .Clk    (clk),
        .Enable (cpu_ce),
        .Rdy    (~pause_cpu),
        .Abort_n(1'b1),

        .IRQ_n  (~(apu_irq | mapper_irq)),
        .NMI_n  (~nmi),
        .SO_n   (1'b1),
        .R_W_n  (cpu_rnw),
        .Sync(), .EF(), .MF(), .XF(), .ML_n(), .VP_n(), .VDA(), .VPA(),

        .A      (cpu_addr),
        .DI     (cpu_rnw ? from_data_bus : cpu_dout),
        .DO     (cpu_dout),

        .Regs(), .DEBUG(), .NMI_ack()
    );

    /**********************************************************/
    /*************              DMA             ***************/
    /**********************************************************/

    wire [15:0] dma_aout;
    wire dma_aout_enable;
    wire dma_read;
    wire [7:0] dma_data_to_ram;
    wire apu_dma_request, apu_dma_ack;
    wire [15:0] apu_dma_addr;

    // Determine the values on the bus outgoing from the CPU chip (after DMA / APU)
    wire [15:0] addr = dma_aout_enable ? dma_aout  : cpu_addr;
    wire [7:0]  dbus = dma_aout_enable ? dma_data_to_ram : cpu_dout;
    wire mr_int      = dma_aout_enable ? dma_read  : cpu_rnw;
    wire mw_int      = dma_aout_enable ? !dma_read : !cpu_rnw;

    DmaController dma(
        .clk            (clk),
        .ce             (cpu_ce),
        .reset          (reset),
        .odd_cycle      (odd_or_even),                // Even or odd cycle
        .sprite_trigger ((addr == 'h4014 && mw_int)), // Sprite trigger
        .dmc_trigger    (apu_dma_request),            // DMC Trigger
        .cpu_read       (cpu_rnw),                    // CPU in a read cycle?
        .data_from_cpu  (cpu_dout),                   // Data from cpu
        .data_from_ram  (from_data_bus),              // Data from RAM etc.
        .dmc_dma_addr   (apu_dma_addr),               // DMC addr
        .aout           (dma_aout),
        .aout_enable    (dma_aout_enable),
        .read           (dma_read),
        .data_to_ram    (dma_data_to_ram),
        .dmc_ack        (apu_dma_ack),
        .pause_cpu      (pause_cpu)
    );

    /**********************************************************/
    /*************             APU              ***************/
    /**********************************************************/

    wire apu_cs = addr >= 'h4000 && addr < 'h4018;
    wire [7:0] apu_dout;
    wire [15:0] sample_apu;
    wire NES_APU_enhancements_ce;

    APU apu(
        .MMC5           (1'b0),
        .clk            (clk),
        .PHI2           (phi2),
        .CS             (apu_cs),
        .PAL            (sys_type[0]),
        .ce             (apu_ce),
        .reset          (reset),
        .cold_reset     (cold_reset),
        .ADDR           (addr[4:0]),
        .DIN            (dbus),
        .DOUT           (apu_dout),
        .RW             (cpu_rnw),
        .audio_channels (audio_channels),
        .Sample         (sample_apu),
        .DmaReq         (apu_dma_request),
        .DmaAck         (apu_dma_ack),
        .DmaAddr        (apu_dma_addr),
        .DmaData        (from_data_bus),
        .odd_or_even    (odd_or_even),
        .IRQ            (apu_irq),
        .allow_us       (1'b0)
    );

    /**********************************************************/
    /*************             PPU              ***************/
    /**********************************************************/

    // The real PPU has a CS pin which is a combination of the output of the 74319 (ppu address selector)
    // and the M2 pin from the CPU. This will only be low for 1 and 7/8th PPU cycles, or
    // 7 and 1/2 master cycles on NTSC. Therefore, the PPU should read or write once per cpu cycle, and
    // with our alignment, this should occur at PPU cycle 2 (the *third* cycle).

    wire mr_ppu     = mr_int && ppu_read;               // Read *from* the PPU.
    wire mw_ppu     = mw_int && ppu_write;              // Write *to* the PPU.
    wire ppu_cs = addr >= 'h2000 && addr < 'h4000;
    wire [7:0] ppu_dout;                                // Data from PPU to CPU
    wire chr_read, chr_write;                           // If PPU reads/writes from VRAM
    wire [13:0] chr_addr;                               // Address PPU accesses in VRAM
    wire [7:0] chr_from_ppu;                            // Data from PPU to VRAM
    wire [7:0] chr_to_ppu;
    wire [19:0] mapper_ppu_flags;                       // PPU flags for mapper cheating
    wire [8:0] ppu_cycle;
    assign cycle = use_fake_h ? 9'd340 : ppu_cycle;

    PPU ppu(
        .clk              (clk),
        .ce               (ppu_ce),
        .reset            (reset),
        .sys_type         (sys_type),
        .color            (color),
        .din              (dbus),
        .dout             (ppu_dout),
        .ain              (addr[2:0]),
        .read             (ppu_cs && mr_ppu),
        .write            (ppu_cs && mw_ppu),
        .nmi              (nmi),
        .vram_r           (chr_read),
        .vram_w           (chr_write),
        .vram_a           (chr_addr),
        .vram_din         (chr_to_ppu),
        .vram_dout        (chr_from_ppu),
        .scanline         (scanline),
        .cycle            (ppu_cycle),
        .mapper_ppu_flags (mapper_ppu_flags),
        .emphasis         (emphasis),
        .short_frame      (skip_pixel)
    );


endmodule
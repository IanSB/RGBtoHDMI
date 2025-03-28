#-------------------------------------------------------------------------
# VideoCore IV implementation of RGBtoHDMI
# (c) IanB Nov 2021
#-------------------------------------------------------------------------

# GPIO registers

.equ GPU_COMMAND,          0x7e0000a0  #use MBOX0-MBOX7 for ARM communications
.equ GPU_DATA_BUFFER_0,    0x7e0000a4
.equ GPU_DATA_BUFFER_1,    0x7e0000a8
.equ GPU_DATA_BUFFER_2,    0x7e0000ac
.equ GPU_DATA_BUFFER_3,    0x7e0000b0
.equ GPU_DATA_BUFFER_4,    0x7e0000b4
.equ GPU_DATA_BUFFER_5,    0x7e0000b8
.equ GPU_DATA_BUFFER_5,    0x7e0000bc

.equ GPU_COMMAND_offset,   0
.equ DATA_BUFFER_0_offset, 4
.equ DATA_BUFFER_1_offset, 8
.equ DATA_BUFFER_2_offset, 12
.equ DATA_BUFFER_3_offset, 16
.equ DATA_BUFFER_4_offset, 20
.equ DATA_BUFFER_5_offset, 24
.equ DATA_BUFFER_6_offset, 28

.equ GPLEV0,            0x7e200034
.equ GPEDS0,            0x7e200040
.equ GPREN0,            0x7e20004c
.equ GPFEN0,            0x7e200058

.equ INTEN,             0x7e00B210       # Interrupt enable reg


.equ PLLD_FRAC,         0x7e102240
.equ CM_PASSWORD,       0x5a000000

.equ SMI_BASE,      	0x7e600010
#.equ SMI_BASE,      	0xc8800000

.equ SMI_CTRL,      	0x00
.equ SMI_CTRL_BIT_RUN,  0
.equ SMI_CTRL_BIT_PLL_0, 1
.equ SMI_CTRL_BIT_PLL_1, 2
.equ SMI_CTRL_BIT_DMA,  3
.equ SMI_CTRL_BIT_MONO_LEFT,  4
.equ SMI_CTRL_BIT_MONO_RIGHT, 5

.equ SMI_STATUS,      	0x04
.equ SMI_STATUS_BIT_RUNNING,   0

.equ SMI_DEFAULT_PLLDFRAC, 8
.equ SMI_BUFFER_START, 12
.equ SMI_BUFFER_END, 16
.equ SMI_ERROR_COUNT, 20
.equ SMI_DROP_REPEAT_COUNT, 24
.equ SMI_DMA_OFFSET, 28

.equ DMA0POINTER,       0x7e00700C

.equ HDMI_MAI_DATA_BUS, 0x7E808020        # pi 4 = 0x7EF2001C
.equ HD_MAI_CTL,        0x7e808014
.equ HD_MAI_CTL_EMPTY,	10
.equ HD_MAI_CTL_FULL,   11
.equ HD_MAI_CTL_ERRORF, 1
.equ HD_MAI_CTL_ERRORE, 2


.equ PLL_OFFSET_SLOW, 0x04
.equ PLL_OFFSET_FAST, 0x40

.equ MAX_DMA_DRIFT, 112
.equ MAX_DMA_DRIFT_ERROR, 144

.equ REPEAT_22Khz, 0

.equ CLOCK_BIT,       15 #GPIO clock bit (RxD)
.equ LOG_FRAME_ERROR, 25
#.equ PLL_SET_LOW_2, 26
#.equ PLL_SET_HIGH_2, 27
#.equ PLL_SET_LOW_1, 28
#.equ PLL_SET_HIGH_1, 29
.equ DROP_SAMPLE, 30
.equ REPEAT_SAMPLE, 31

.equ SYNC_CHECK_RATE, 192
.equ BUFFER_CHECK_COUNT, SYNC_CHECK_RATE * 4



.equ IEC958_FRAMES_PER_BLOCK,     192
.equ PLL_LOOP_COUNT, IEC958_FRAMES_PER_BLOCK * 8
.equ IEC958_STATUS_BYTES,         5
.equ IEC958_B_FRAME_PREAMBLE,     0x0f # 0x08 in linux and 0x0f in circle

#flag bits  31,30, 17,16 15,14 1,0
.equ FINAL_BIT,            31             #signal if this sample word is the last
#bit 30 is second copy of ALT_MUX_BIT
.equ PSYNC_BIT,            17             #alternates on each full 6 word buffer
.equ ODD_EVEN_BIT_HI,      16             #signal if low or high 16 bit sample is to be used
#bit 15 unused
.equ ALT_MUX_BIT,          14             #moved version of MUX bit also at bit position 30
.equ ODD_EVEN_BIT_LO,      0              #signal if low or high 16 bit sample is to be used

.equ DEFAULT_BIT_STATE,    0x00020001     #FINAL_BIT=0, PSYNC_BIT=1, ODD_EVEN_BIT_HI=0, ODD_EVEN_BIT_LO=1

#GPIO bits
.equ MUX_BIT,              24             #video input on MUX bit for FFOSD
.equ SYNC_BIT,             23             #sync input
.equ VIDEO_MASK,           0x3ffc         #12bit GPIO mask

#command bits
.equ TERMINATE_FLAG,       31
.equ SYNC_ABORT_FLAG,      30
#bits 20-22 are simple sync type
.equ LEADING_SYNC_FLAG,    16
.equ SIMPLE_SYNC_FLAG,     15
.equ HIGH_LATENCY_FLAG,    14
.equ OLD_FIRMWARE_FLAG,    13

.equ COMMAND_MASK,         0x00000fff     #masks out command bits that trigger sync detection
#macros

.macro    USE_NOP
nop
nop
nop

.endm

.macro LO_PSYNC_CAPTURE
wait_psync_lo\@:
   ld     r0, (r4)
   USE_NOP
   btst   r0, PSYNC_BIT
   bne    wait_psync_lo\@
   btst   r0, MUX_BIT
   and    r0, r6
   bsetne r0, ALT_MUX_BIT  #move mux bit to position in 16 bit sample
   sub    r3, 1
   or     r0, r2           #merge bit state
.endm

.macro HI_PSYNC_CAPTURE
wait_psync_hi\@:
   ld     r1, (r4)
   USE_NOP
   btst   r1, PSYNC_BIT
   beq    wait_psync_hi\@
   btst   r1, MUX_BIT
   and    r1, r6
   bsetne r1, ALT_MUX_BIT  #move mux bit to position in 16 bit sample
   lsl    r1, 16           #merge lo and hi samples
   cmp    r3, 0
   or     r0, r1
.endm


.macro OFW_LO_PSYNC_CAPTURE
wait_psync_lo\@:
   ld     r0, (r4)
   USE_NOP
   btst   r0, PSYNC_BIT
   bne    wait_psync_lo\@
   ld     r0, (r4)
   btst   r0, MUX_BIT
   and    r0, r6
   bsetne r0, ALT_MUX_BIT  #move mux bit to position in 16 bit sample
   sub    r3, 1
   or     r0, r2           #merge bit state
.endm

.macro OFW_HI_PSYNC_CAPTURE
wait_psync_hi\@:
   ld     r1, (r4)
   USE_NOP
   btst   r1, PSYNC_BIT
   beq    wait_psync_hi\@
   ld     r1, (r4)
   btst   r1, MUX_BIT
   and    r1, r6
   bsetne r1, ALT_MUX_BIT  #move mux bit to position in 16 bit sample
   lsl    r1, 16           #merge lo and hi samples
   cmp    r3, 0
   or     r0, r1
.endm


.macro HL_LO_PSYNC_CAPTURE
wait_psync_lo\@:
   ld     r0, (r4)
   USE_NOP
   btst   r0, PSYNC_BIT
   bne    wait_psync_lo\@
   btst   r0, MUX_BIT
   and    r0, r6
   bsetne r0, ALT_MUX_BIT  #move mux bit to position in 16 bit sample

.endm

.macro HL_HI_PSYNC_CAPTURE
wait_psync_hi\@:
   ld     r1, (r4)
   USE_NOP
   btst   r1, PSYNC_BIT
   beq    wait_psync_hi\@
   btst   r1, MUX_BIT
   and    r1, r6
   bsetne r1, ALT_MUX_BIT  #move mux bit to position in 16 bit sample
   lsl    r1, 16           #merge lo and hi samples
   or     r0, r1
.endm


.macro EDGE_DETECT
waitPSE\@:
   ld     r0, (r4)
   USE_NOP
   eor    r0, r2
   btst   r0, PSYNC_BIT
   bne    waitPSE\@
   eor    r0, r2       #restore r0 value
   bchg   r2, PSYNC_BIT
.endm


# main code entry point
   di
   b vpu0
   .align 2
   di
   b vpu1
   b vpu1_interrupt
vpu0:
   cmp    r0, 1
   bne    not_gpio_read_benchmark
   mov    r2, 100000
   mov    r1, GPLEV0
read_bench_loop:
   ld     r3, (r1)  #read gpio
   sub    r2, 1
   cmp    r2, 0
   bne    read_bench_loop
   ei
   rts

not_gpio_read_benchmark:
   cmp    r0, 2
   bne    not_mbox_write_benchmark
   mov    r2, 100000
   mov    r1, GPU_DATA_BUFFER_5
   mov    r3, 0
write_bench_loop:
   st     r3, (r1)  #write to mbox
   sub    r2, 1
   cmp    r2, 0
   bne    write_bench_loop
   ei
   rts

exit:
   ei
   pop   r0-r15,pc

not_mbox_write_benchmark:
   push   r0-r15,lr
   mov    r4, GPLEV0
   mov    r5, GPU_COMMAND
   mov    r6, VIDEO_MASK
   mov    r7, COMMAND_MASK
   mov    r8, DEFAULT_BIT_STATE
   mov    r12, 0                       # remains at zero for rest of the code
   mov    r13, 1                       # remains at 1 for rest of the code
   st     r12, DATA_BUFFER_0_offset(r5)
   st     r12, DATA_BUFFER_1_offset(r5)
   st     r12, DATA_BUFFER_2_offset(r5)
   st     r12, DATA_BUFFER_3_offset(r5)
   st     r12, DATA_BUFFER_4_offset(r5)
   st     r12, DATA_BUFFER_5_offset(r5)
   st     r12, DATA_BUFFER_6_offset(r5)

wait_for_command:
   ld     r0, DATA_BUFFER_0_offset(r5)
   ld     r1, DATA_BUFFER_1_offset(r5)
   ld     r2, DATA_BUFFER_2_offset(r5)
   ld     r3, DATA_BUFFER_3_offset(r5)
   ld     r9, DATA_BUFFER_4_offset(r5)
   ld     r10, DATA_BUFFER_5_offset(r5)
   ld     r11, DATA_BUFFER_6_offset(r5)
   st     r12, GPU_COMMAND_offset(r5)    #set command register to 0
   bset   r0, FINAL_BIT
   bset   r1, FINAL_BIT
   bset   r2, FINAL_BIT
   bset   r3, FINAL_BIT
   bset   r9, FINAL_BIT
   bset   r10, FINAL_BIT
   bset   r11, FINAL_BIT
   st     r0, DATA_BUFFER_0_offset(r5)
   st     r1, DATA_BUFFER_1_offset(r5)
   st     r2, DATA_BUFFER_2_offset(r5)
   st     r3, DATA_BUFFER_3_offset(r5)
   st     r9, DATA_BUFFER_4_offset(r5)
   st     r10, DATA_BUFFER_5_offset(r5)
   st     r11, DATA_BUFFER_6_offset(r5)

   mov    r2, r8                        #set the default state of the control bits

wait_for_command_loop:
   nop    #some idle time to reduce continuous polling of register
   ld     r3, GPU_COMMAND_offset(r5)
   nop
   cmp    r3, 0
   nop
   beq    wait_for_command_loop
   btst   r3, TERMINATE_FLAG
   bne    exit
   btst   r3, SYNC_ABORT_FLAG
   bne    wait_for_command
   btst   r3, SIMPLE_SYNC_FLAG                   #bit signals bits 20-22 are a sync command
   beq    do_capture
   mov    r1, r3
   lsr    r1, 20
   and    r1, 0x07

   #simple mode sync detection, enters with PSYNC_BIT set in r2
   cmp    r1, 0
   beq    edge_trail_neg
   cmp    r1, 1
   beq    edge_lead_neg
   bclr   r2, PSYNC_BIT             #only +ve edge (inverted later)
   cmp    r1, 2
   beq    edge_trail_pos
   cmp    r1, 3
   beq    edge_lead_pos
   cmp    r1, 4
   beq    edge_trail_both
   cmp    r1, 5
   bne    wait_for_command
   #if here then edge_lead_both

edge_lead_both:
   EDGE_DETECT
   btst   r0, SYNC_BIT
   bne    edge_lead_both
   st     r13, DATA_BUFFER_0_offset(r5)   #lsbit flags sync detected
   b      done_simple_sync

edge_trail_both:
   EDGE_DETECT
   btst   r0, SYNC_BIT
   bne    edge_trail_both
   st     r13, DATA_BUFFER_0_offset(r5)   #lsbit flags sync detected
edge_trail_both_hi:
   EDGE_DETECT
   btst   r0, SYNC_BIT
   beq    edge_trail_both_hi
   b      done_simple_sync

edge_lead_neg:
edge_lead_pos:
   #incoming psync state controls edge
wait_csync_lo2:
   EDGE_DETECT
   EDGE_DETECT
   btst   r0, SYNC_BIT
   bne    wait_csync_lo2
   st     r13, DATA_BUFFER_0_offset(r5)   #lsbit flags sync detected
   b      done_simple_sync

edge_trail_neg:
edge_trail_pos:
   #incoming psync state controls edge *** this one used by amiga
wait_csync_lo:
   EDGE_DETECT
   EDGE_DETECT
   btst   r0, SYNC_BIT
   bne    wait_csync_lo
   st     r13, DATA_BUFFER_0_offset(r5)   #lsbit flags sync detected
wait_csync_hi:
   EDGE_DETECT
   EDGE_DETECT
   btst   r0, SYNC_BIT
   beq    wait_csync_hi

done_simple_sync:
   btst   r2, PSYNC_BIT
   bne    no_compensate_psync
   EDGE_DETECT           #have to compensate because capture hard coded to always start on same edge
no_compensate_psync:
   mov    r2, r8         #set the default state of the control bits
   b      capture_rest

do_capture:
   btst   r3, OLD_FIRMWARE_FLAG         #bit signals old firmware capture, requires double reads as psync not pipelined
   bne    ofw_capture

wait_csync_lo_cpld:
   ld     r0, GPU_COMMAND_offset(r5)
   btst   r0, SYNC_ABORT_FLAG
   bne    capture_rest
   ld     r0, (r4)
   USE_NOP
   btst   r0, SYNC_BIT
   bne    wait_csync_lo_cpld

   btst   r3, LEADING_SYNC_FLAG
   bne    capture_rest

wait_csync_hi_cpld:
   ld     r0, GPU_COMMAND_offset(r5)
   btst   r0, SYNC_ABORT_FLAG
   bne    capture_rest
   ld     r0, (r4)
   USE_NOP
   btst   r0, SYNC_BIT
   beq    wait_csync_hi_cpld

capture_rest:
   btst   r3, HIGH_LATENCY_FLAG         #bit signals high latency capture, only suitable for 9/12bpp modes
   bne    hl_capture

   and    r3, r7         #mask off any command bits (max capture is 4095 psync cycles)
   add    r3, 1          #round up to multiple of 2
   lsr    r3, 1          #divide by 2 as capturing 2 samples per cycle

capture_loop:
   LO_PSYNC_CAPTURE
   HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_0_offset(r5)
   beq    wait_for_command

   LO_PSYNC_CAPTURE
   HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_1_offset(r5)
   beq    wait_for_command

   LO_PSYNC_CAPTURE
   HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_2_offset(r5)
   beq    wait_for_command

   LO_PSYNC_CAPTURE
   HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_3_offset(r5)
   beq    wait_for_command

   LO_PSYNC_CAPTURE
   HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_4_offset(r5)
   beq    wait_for_command

   LO_PSYNC_CAPTURE
   HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_5_offset(r5)
   beq    wait_for_command

   LO_PSYNC_CAPTURE
   bchg   r2, PSYNC_BIT        #invert the software psync bit every 12 samples / 6 words
   HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_6_offset(r5)
   beq    wait_for_command

   b      capture_loop

ofw_capture:
ofw_wait_csync_lo_cpld:
   ld     r0, GPU_COMMAND_offset(r5)
   btst   r0, SYNC_ABORT_FLAG
   bne    ofw_capture_rest
   ld     r0, (r4)
   USE_NOP
   btst   r0, SYNC_BIT
   bne    ofw_wait_csync_lo_cpld
   ld     r0, (r4)
   USE_NOP
   btst   r0, SYNC_BIT
   bne    ofw_wait_csync_lo_cpld
   ld     r0, (r4)
   USE_NOP
   btst   r0, SYNC_BIT
   bne    ofw_wait_csync_lo_cpld
   ld     r0, (r4)
   USE_NOP
   btst   r0, SYNC_BIT
   bne    ofw_wait_csync_lo_cpld
   ld     r0, (r4)
   USE_NOP
   btst   r0, SYNC_BIT
   bne    ofw_wait_csync_lo_cpld

   btst   r3, LEADING_SYNC_FLAG
   bne    ofw_capture_rest

ofw_wait_csync_hi_cpld:
   ld     r0, GPU_COMMAND_offset(r5)
   btst   r0, SYNC_ABORT_FLAG
   bne    ofw_capture_rest
   ld     r0, (r4)
   btst   r0, SYNC_BIT
   beq    ofw_wait_csync_hi_cpld
   ld     r0, (r4)
   btst   r0, SYNC_BIT
   beq    ofw_wait_csync_hi_cpld
   ld     r0, (r4)
   btst   r0, SYNC_BIT
   beq    ofw_wait_csync_hi_cpld
   ld     r0, (r4)
   btst   r0, SYNC_BIT
   beq    ofw_wait_csync_hi_cpld
   ld     r0, (r4)
   btst   r0, SYNC_BIT
   beq    ofw_wait_csync_hi_cpld

ofw_capture_rest:
   and    r3, r7         #mask off any command bits (max capture is 4095 psync cycles)
   add    r3, 1          #round up to multiple of 2
   lsr    r3, 1          #divide by 2 as capturing 2 samples per cycle

old_firmware_capture_loop:
   OFW_LO_PSYNC_CAPTURE
   OFW_HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_0_offset(r5)
   beq    wait_for_command

   OFW_LO_PSYNC_CAPTURE
   OFW_HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_1_offset(r5)
   beq    wait_for_command

   OFW_LO_PSYNC_CAPTURE
   OFW_HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_2_offset(r5)
   beq    wait_for_command

   OFW_LO_PSYNC_CAPTURE
   OFW_HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_3_offset(r5)
   beq    wait_for_command

   OFW_LO_PSYNC_CAPTURE
   OFW_HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_4_offset(r5)
   beq    wait_for_command

   OFW_LO_PSYNC_CAPTURE
   OFW_HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_5_offset(r5)
   beq    wait_for_command

   OFW_LO_PSYNC_CAPTURE
   bchg   r2, PSYNC_BIT        #invert the software psync bit every 12 samples / 6 words
   OFW_HI_PSYNC_CAPTURE

   st     r0, DATA_BUFFER_6_offset(r5)
   beq    wait_for_command

   b      old_firmware_capture_loop

hl_capture:
   and    r3, r7           #mask off any command bits (max capture is 4095 psync cycles)
   mov    r0, r3
   add    r0, 13           #round up to multiple of 14
   mov    r1, 14
   divu   r3, r0, r1       #divide by 14 as capturing 14 samples per cycle
   bchg   r2, PSYNC_BIT    #pre invert the software psync bit

high_latency_capture_loop:
   HL_LO_PSYNC_CAPTURE
   bchg   r2, PSYNC_BIT    #invert the software psync bit every 12 samples / 6 words
   or     r0, r2           #merge bit state
   HL_HI_PSYNC_CAPTURE
   or     r0, r1
   st     r0, DATA_BUFFER_0_offset(r5)

   HL_LO_PSYNC_CAPTURE
   or     r0, r2           #merge bit state
   HL_HI_PSYNC_CAPTURE
   or     r0, r1
   st     r0, DATA_BUFFER_1_offset(r5)

   HL_LO_PSYNC_CAPTURE
   sub    r3, 1
   or     r0, r2           #merge bit state
   HL_HI_PSYNC_CAPTURE
   or     r0, r1
   st     r0, DATA_BUFFER_2_offset(r5)

   HL_LO_PSYNC_CAPTURE
   or     r0, r2           #merge bit state
   HL_HI_PSYNC_CAPTURE
   or     r0, r1
   st     r0, DATA_BUFFER_3_offset(r5)

   HL_LO_PSYNC_CAPTURE
   or     r0, r2           #merge bit state
   HL_HI_PSYNC_CAPTURE
   or     r0, r1
   st     r0, DATA_BUFFER_4_offset(r5)

   HL_LO_PSYNC_CAPTURE
   or     r0, r2           #merge bit state
   HL_HI_PSYNC_CAPTURE
   or     r0, r1
   st     r0, DATA_BUFFER_5_offset(r5)

   HL_LO_PSYNC_CAPTURE
   or     r0, r2           #merge bit state
   HL_HI_PSYNC_CAPTURE
   cmp    r3, 0
   or     r0, r1
   st     r0, DATA_BUFFER_6_offset(r5)

   bne    high_latency_capture_loop

   b      wait_for_command

#*****************************************************************************************
#macros for audio capture
.macro DELAY_NOP
   nop
   nop
   nop
   nop

.endm


.macro WAIT_EDGE_FOR_DATA_BIT
waitBCH\@:
   DELAY_NOP
   ld     r0, (r4)
   eor    r0, r2
   btst   r0, CLOCK_BIT
   beq    waitBCH\@
   ld     r0, (r4)          #second read for reliability
   bchg   r2, CLOCK_BIT     #edge detect if using 3 gpios otherwise high detect
   btst   r0, r15 # DATABIT
   addne  r8, 1  #parity count
.endm

.macro WAIT_EDGE_FOR_LR_BIT
waitBCH\@:
   DELAY_NOP
   ld     r0, (r4)
   eor    r0, r2
   btst   r0, CLOCK_BIT
   beq    waitBCH\@
   ld     r0, (r4)          #second read for reliability
   bchg   r2, CLOCK_BIT     #edge detect if using 3 gpios otherwise high detect
   btst   r0, r16 # LRBIT
.endm


.macro LO_BITCLK
waitBCL\@:
   ld     r0, (r4)
   btst   r0, CLOCK_BIT
   beq    waitBCL\@
.endm

.macro HI_BITCLK_FOR_DATA_BIT
waitBCH\@:
   ld     r0, (r4)
   btst   r0, CLOCK_BIT
   bne    waitBCH\@
   ld     r0, (r4)          #second read for reliability
   btst   r0, r15 # DATABIT
   addne  r8, 1  #parity count
.endm

.macro LO_BITCLK_FOR_LR_BIT
waitBCL\@:
   ld     r0, (r4)
   btst   r0, CLOCK_BIT
   beq    waitBCL\@
   ld     r0, (r4)   #second read for delay to allow LR to stabilise in two gpio mode
   btst   r0, r16 # LRBIT
.endm



.macro HI_BITCLK
waitBCHO\@:
   ld     r0, (r4)
   btst   r0, CLOCK_BIT
   bne    waitBCHO\@
.endm

.macro WAIT_FOR_DATA_BIT
.if TWOGPIO == 1
   LO_BITCLK
   HI_BITCLK_FOR_DATA_BIT
.else
   WAIT_EDGE_FOR_DATA_BIT
.endif
.endm

.macro WAIT_FOR_LR_BIT
.if TWOGPIO == 1
   LO_BITCLK_FOR_LR_BIT
.else
   WAIT_EDGE_FOR_LR_BIT
.endif
.endm

.macro WAIT_HI_TWOGPIO_ONLY
.if TWOGPIO == 1
   HI_BITCLK
.endif
.endm

.macro LO_LRCLK
.if TWOGPIO == 1
waitLRCL2\@:
   HI_BITCLK
   LO_BITCLK_FOR_LR_BIT
   bne    waitLRCL2\@
   HI_BITCLK
.else
waitLRCL\@:
   WAIT_EDGE_FOR_LR_BIT
   bne    waitLRCL\@
.endif
.endm


.macro HI_LRCLK
.if TWOGPIO == 1
waitLRCH2\@:
   HI_BITCLK
   LO_BITCLK_FOR_LR_BIT
   beq    waitLRCH2\@
   HI_BITCLK
.else
waitLRCH\@:
   WAIT_EDGE_FOR_LR_BIT
   beq    waitLRCH\@
.endif
.endm




.macro IEC958_STATUS

    #byte 0 = 0x04   - bits 0-7,   bit 2 set   // consumer, PCM, no copyright, no pre-emphasis
    #byte 1 = 0x00   - bits 8-15                // category (general mode)
    #byte 2 = 0x00   - bits 16-23              // source number, take no account of channel number
    #byte 3 = 0x02   - bits 24-31  bit 25 set    // sampling frequency uchFS = 2 = 48000
    #byte 4 = 0xDB   - bits 32-39  0b1011 | (13 << 4) bits 1101 1011   bits 32, 33, 35, 36, 38, 39      // 24 bit samples, original freq.  uchOrigFS = 13 = 48000

    cmp    r6, 0
    oreq   r1, IEC958_B_FRAME_PREAMBLE
    cmp r6, (IEC958_STATUS_BYTES * 8)
    bge    no_status_bytes\@
    cmp   r6, 2
    cmpne r6, 25
    bseteq r1, 30
    sub   r6, 32
    cmp   r6, (32 - 32)
    cmpne r6, (33 - 32)
    cmpne r6, (35 - 32)
    cmpne r6, (36 - 32)
    cmpne r6, (38 - 32)
    cmpne r6, (39 - 32)
    bseteq r1, 30
    add   r6, 32
no_status_bytes\@:
    mov    r8, 0
    btst   r1, 31  #test existing parity count
    movne  r8, 1
    bclr   r1, 31

    btst   r1, 0
    addne  r8, 1
    btst   r1, 1
    addne  r8, 1
    btst   r1, 2
    addne  r8, 1
    btst   r1, 3
    addne  r8, 1
    btst   r1, 28
    addne  r8, 1
    btst   r1, 29
    addne  r8, 1
    btst   r1, 30
    addne  r8, 1

    btst   r8, 0
    bsetne r1, 31   # set parity bit count is odd

.endm

.macro WRITE
    st     r1, (r5)
#increment buffer pointer
    add    r5, 4
    cmp    r5, r14
    movge  r5, r13
.endm

SINGLE_WRITE_LEFT_RIGHT:
   btst   r21, SMI_CTRL_BIT_MONO_RIGHT
   movne  r1, r12
   moveq  r1, r11
   IEC958_STATUS
   WRITE
   btst   r21, SMI_CTRL_BIT_MONO_LEFT
   movne  r1, r11
   moveq  r1, r12
   IEC958_STATUS
   WRITE
   add    r6, 1
   cmp    r6, IEC958_FRAMES_PER_BLOCK
   movge  r6, 0
   rts

.macro CHECK_UNDERRUN
   # check for underrun
   cmp    r9,0x10
   bne    not_empty\@             #only check once per frame
   btst   r2, REPEAT_SAMPLE
   bne    not_empty\@          #repeat pending

   btst   r21, SMI_CTRL_BIT_DMA
   beq  not_dma_underrun\@

   ld     r0, (r10) #get dma pointer
   sub    r3, r14, r13  #buffer size
   cmp    r5, r0
   subge  r1, r5, r0
   bge    got_offset_under\@
   sub    r1, r0, r5
   sub    r1, r3, r1
got_offset_under\@:
   lsr    r3, 1
   sub    r1, r3
   st     r1, SMI_DMA_OFFSET(r20)
   rsb    r1, 0
   cmp    r1, MAX_DMA_DRIFT_ERROR
   bgt    repeat_empty\@
   cmp    r1, MAX_DMA_DRIFT
   bgt    empty\@
   b      not_empty\@

not_dma_underrun\@:
   ld     r0, (r7)  #read MAI CTL
   btst   r0, HD_MAI_CTL_ERRORE
   bne    repeat_empty\@
   btst   r0, HD_MAI_CTL_EMPTY
   beq    not_empty\@
empty\@:
   btst   r21, SMI_CTRL_BIT_PLL_1
   beq    repeat_empty\@
   btst   r21, SMI_CTRL_BIT_PLL_0
   beq    use_slow_pll_empty\@

#   mov    r0, r19
#   sub    r0, PLL_OFFSET_FAST
   sub    r19, PLL_OFFSET_FAST
   mov    r0, r19
   or     r0, CM_PASSWORD
   st     r0, (r18)
   b      not_empty\@

use_slow_pll_empty\@:
   sub    r19, PLL_OFFSET_SLOW
   mov    r0, r19
   or     r0, CM_PASSWORD
   st     r0, (r18)
   b      not_empty\@

repeat_empty\@:
   bset   r2, REPEAT_SAMPLE
   add    r23, 0x10000
   bclr   r23, 31   #max 32767
   st     r23, SMI_DROP_REPEAT_COUNT(r20)
not_empty\@:

.endm

.macro CHECK_OVERRUN
   # check for overrun
   cmp    r9, 0x20           #only check once per frame
   bne    not_full\@
   btst   r2, DROP_SAMPLE
   bne    not_full\@    #drop pending
   btst   r21, SMI_CTRL_BIT_DMA
   beq  not_dma_overrun\@

   ld     r0, (r10) #get dma pointer
   sub    r3, r14, r13  #buffer size
   cmp    r5, r0
   subge  r1, r5, r0
   bge    got_offset_over\@
   sub    r1, r0, r5
   sub    r1, r3, r1
got_offset_over\@:
   lsr    r3, 1
   sub    r1, r3
   st     r1, SMI_DMA_OFFSET(r20)
   cmp    r1, MAX_DMA_DRIFT_ERROR
   bgt    drop_full\@
   cmp    r1, MAX_DMA_DRIFT
   bgt    full\@
   b      not_full\@

not_dma_overrun\@:
   ld     r0, (r7)  #read MAI CTL
   btst   r0, HD_MAI_CTL_ERRORF
   bne    drop_full\@
   btst   r0, HD_MAI_CTL_FULL
   beq    not_full\@
full\@:
   btst   r21, SMI_CTRL_BIT_PLL_1
   beq    drop_full\@
   btst   r21, SMI_CTRL_BIT_PLL_0
   beq    use_slow_pll_full\@

#   mov    r0, r19
#   add    r0, PLL_OFFSET_FAST
   add    r19, PLL_OFFSET_FAST
   mov    r0, r19
   or     r0, CM_PASSWORD
   st     r0, (r18)
   b      not_full\@

use_slow_pll_full\@:
   add    r19, PLL_OFFSET_SLOW
   mov    r0, r19
   or     r0, CM_PASSWORD
   st     r0, (r18)
   b      not_full\@

drop_full\@:
   bset   r2, DROP_SAMPLE
   add    r23, 1
   bclr   r23, 15  #max 32767
   st     r23, SMI_DROP_REPEAT_COUNT(r20)
not_full\@:

.endm


.macro WRITE_LEFT_RIGHT
   bl     SINGLE_WRITE_LEFT_RIGHT
   btst   r2, REPEAT_22Khz
   beq    no_repeat_sample\@
   bl     SINGLE_WRITE_LEFT_RIGHT
no_repeat_sample\@:
.endm


.macro CAPTURE_AUDIO
   HI_LRCLK
   LO_LRCLK
   b  audio_main_loop\@
bad_sync_LR_high1\@:
#WRITE_LEFT_RIGHT  using causes PLL trouble
   LO_LRCLK
   WRITE_LEFT_RIGHT
   b  increment_error\@

bad_sync_LR_low1\@:
#WRITE_LEFT_RIGHT  using causes PLL trouble
   HI_LRCLK
   LO_LRCLK
   WRITE_LEFT_RIGHT
   b  increment_error\@

bad_sync_LR_low2\@:
   WRITE_LEFT_RIGHT   #removing causes PLL trouble
   HI_LRCLK
   LO_LRCLK
   WRITE_LEFT_RIGHT
   b  increment_error\@

bad_sync_LR_high2\@:
   WRITE_LEFT_RIGHT
   LO_LRCLK
   WRITE_LEFT_RIGHT

increment_error\@:
   add    r22, 1
   bset   r2, LOG_FRAME_ERROR
audio_main_loop\@:

   #capture first sample
   #LR clock has just gone high
   btst   r21, SMI_CTRL_BIT_RUN
   beq    abort_audio            #destination not in macro

   mov    r3, 24
   mov    r1, 0
   mov    r8, 0 #parity count
firstloop\@:
   lsl    r1, 1
   WAIT_FOR_DATA_BIT
   orne   r1, 1
   sub    r3, 1
   cmp    r3, 0
   bne    firstloop\@
   lsl    r1, 4
   btst   r8, 0
   bsetne r1, 31   # temp set parity bit if count is odd

   mov    r17, r1   #temp save

   WAIT_FOR_DATA_BIT

   CHECK_OVERRUN

   WAIT_FOR_DATA_BIT

   cmp    r9, BUFFER_CHECK_COUNT >> 1
   bne    skip_update\@
   ld     r21, SMI_CTRL(r20)      #update the control register
skip_update\@:

   mov    r3, 4
firstremain\@:
   WAIT_FOR_DATA_BIT
   sub    r3, 1
   cmp    r3, 0
   bne    firstremain\@

   cmp    r9, (BUFFER_CHECK_COUNT >> 1) + 0x10
   bne    skip_log\@
   btst   r2, LOG_FRAME_ERROR
   beq    skip_log\@
   st     r22, SMI_ERROR_COUNT(r20)
   bclr   r2, LOG_FRAME_ERROR
skip_log\@:

   WAIT_FOR_LR_BIT
   bne    bad_sync_LR_high1\@
   WAIT_HI_TWOGPIO_ONLY

   WAIT_FOR_LR_BIT
   beq    bad_sync_LR_low1\@
   mov    r1, r17
   mov    r11, r1   #save in case of repeat
   IEC958_STATUS
   mov    r17, r1
   WAIT_HI_TWOGPIO_ONLY

   #capture second sample
   #LR clock has just gone low
   mov    r3, 24
   mov    r1, 0
   mov    r8, 0 #parity count
secondloop\@:
   lsl    r1, 1
   WAIT_FOR_DATA_BIT
   orne   r1, 1
   sub    r3, 1
   cmp    r3, 0
   bne    secondloop\@
   lsl    r1, 4
   btst   r8, 0
   bsetne r1, 31   # temp set parity bit if count is odd
   mov    r24, r1   #temp save

   WAIT_FOR_DATA_BIT

   CHECK_UNDERRUN

   WAIT_FOR_DATA_BIT

   btst   r2, REPEAT_SAMPLE
   beq    no_repeat_main\@
   bl     SINGLE_WRITE_LEFT_RIGHT
   bclr   r2, REPEAT_SAMPLE
no_repeat_main\@:

   mov    r3, 4
secondremain\@:
   WAIT_FOR_DATA_BIT
   sub    r3, 1
   cmp    r3, 0
   bne    secondremain\@

   mov    r1, r24
   IEC958_STATUS
   mov    r0, r24
   mov    r24, r1   #temp save
   mov    r1, r0

   WAIT_FOR_LR_BIT
   beq    bad_sync_LR_low2\@
   WAIT_HI_TWOGPIO_ONLY

   WAIT_FOR_LR_BIT
   bne    bad_sync_LR_high2\@
   mov    r12, r1   #save in case of repeat
   WAIT_HI_TWOGPIO_ONLY

   btst   r2, DROP_SAMPLE
   bclrne r2, DROP_SAMPLE
   bne    drop_main\@

   btst   r21, SMI_CTRL_BIT_MONO_RIGHT
   movne  r1, r24
   moveq  r1, r17
   WRITE
   btst   r21, SMI_CTRL_BIT_MONO_LEFT
   movne  r1, r17
   moveq  r1, r24
   WRITE
   add    r6, 1
   cmp    r6, IEC958_FRAMES_PER_BLOCK
   movge  r6, 0
drop_main\@:
   btst   r2, REPEAT_22Khz
   beq    no_repeat_sample_main\@
   bl     SINGLE_WRITE_LEFT_RIGHT
no_repeat_sample_main\@:

   add    r9, 1
   cmp    r9, BUFFER_CHECK_COUNT
   movge  r9, 0

   b      audio_main_loop\@


.endm

vpu1_interrupt:
   rti

#start of audio capture
vpu1:
audio_capture:
   push   r0-r18,lr

   #r0 = gpio value
   #r1 = sample value
   #r2 = flags reg
   #r3 = counter
   #r4 = gpio address
   #r5 = sample pointer
   #r6 = frame counter
   #r7 = MAI_CTRL pointer
   #r8 = parity count
   #r9 = overrun/underrun check counter
   #r10 = DMA0POINTER
   #r11 = copy of first sample
   #r12 = copy of second sample
   #r13 = low end of dma buffer
   #r14 = high end of dma buffer
   #r15 = audio data pin
   #r16 = audio LR pin
   #r17 = temp save reg
   #r18 = pointer to PLLD FRAC register
   #r19 = original value of PLLD FRAC register
   #r20 = pointer to SMI register block used for ARM communications
   #r21 = current value of ARM control register
   #r22 = frame error count
   #r23 = drop and repeat counts (15 bits each)
   #r24 = temp save reg

# on entry r1 = audio_data_pin, r2 = audio_LR_pin, r3 = sample_repeat)
   mov   r15, r1
   mov   r16, r2
   mov   r2, r3

   mov    r20, SMI_BASE
   mov    r10, DMA0POINTER

   mov    r4, GPLEV0
   mov    r7, HD_MAI_CTL

   mov    r18, PLLD_FRAC

#   mov    r0, (1 << CLOCK_BIT)
#   st     r0, (GPREN0-GPLEV0)(r4)   #enable rising edge detection
#   st     r0, (GPFEN0-GPLEV0)(r4)   #enable falling edge detection

#   mov r1, INTEN
#   ld  r0, (r1)
#   or  r0, (1 << 17)
#   st  r0, (r1)

  # Acknowledge the interrupt
  #  ld     r0, (GPEDS0-GPLEV0)(r4)
  #  st     r0, (GPEDS0-GPLEV0)(r4)

abort_audio:
   mov    r0,0
   st     r0, SMI_CTRL(r20)
   st     r0, SMI_STATUS(r20)

   st     r0, SMI_ERROR_COUNT(r20)
   st     r0, SMI_DROP_REPEAT_COUNT(r20)

   ld     r0, SMI_DEFAULT_PLLDFRAC(r20)
   or     r0, CM_PASSWORD
   st     r0, (r18)
command_loop:
   mov    r0,0
   st     r0, SMI_STATUS(r20)

   mov    r1, 256
sleep:
   DELAY_NOP
   sub   r1, 1
   cmp   r1, 0
   bne   sleep

   ld     r21, SMI_CTRL(r20)      #read the control register
   btst   r21, SMI_CTRL_BIT_RUN
   beq    command_loop

   ld     r19, (r18)

   #set the pll
#   mov    r0, r19
#   btst   r21, SMI_CTRL_BIT_PLL
#   beq    norunfast
#   sub    r0, PLL_OFFSET_SLOW     # run pll fast so buffer fills up
#norunfast:
#   or     r0, CM_PASSWORD
#   st     r0, (r18)


   mov    r6,  0 #nFrame

   mov    r9,  0 #overundercheck counter

   mov    r11, 0 #copy of first sample
   mov    r12, 0 #copy of second sample
   bclr   r2, CLOCK_BIT #flags
   mov    r22, 0 #error count
   mov    r23, 0 # drop/rep count
   mov    r24, 0

   mov    r0, 1
   st     r0, SMI_STATUS(r20)

   # get buffer
   ld     r13, SMI_BUFFER_START(r20)
   ld     r14, SMI_BUFFER_END(r20)

   btst   r21, SMI_CTRL_BIT_DMA
   beq    no_dma
   ld     r5, (r10) #get dma pointer
   sub    r3, r14, r13  #buffer size
   mov    r0, r3
   lsr    r0, 1
   add    r5, r0
   cmp    r5, r3
   subge  r5, r3
   b      use_dma
no_dma:
   mov   r5, r13   #pointer to HDMI_MAI_DATA_BUS in non dma mode
   #put some zero samples in the buffer
   mov r3, 8 #half of buffer which is 0x11 pair writes
fill:
   mov    r11, 0
   mov    r12, 0
   bl     SINGLE_WRITE_LEFT_RIGHT
   sub    r3, 1
   cmp    r3, 0
   bne    fill

use_dma:
   cmp    r15, r16             # if data pin and LR pin are the same then two GPIO capture
   beq    two_gpio_capture

.set TWOGPIO, 0
   CAPTURE_AUDIO

two_gpio_capture:
.set TWOGPIO, 1
   CAPTURE_AUDIO


#   ei
#   pop   r0-r18,pc
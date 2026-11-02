arm_ss = ss.source_set()
arm_ss.add(files('boot.c'), fdt)
arm_ss.add(when: 'CONFIG_ARM_VIRT', if_true: files('virt.c'))
arm_ss.add(when: 'CONFIG_ACPI', if_true: files('virt-acpi-build.c'))
arm_ss.add(when: 'CONFIG_DIGIC', if_true: files('digic_boards.c'))
arm_ss.add(when: 'CONFIG_EMCRAFT_SF2', if_true: files('msf2-som.c'))
arm_ss.add(when: 'CONFIG_HIGHBANK', if_true: files('highbank.c'))
arm_ss.add(when: 'CONFIG_INTEGRATOR', if_true: files('integratorcp.c'))
arm_ss.add(when: 'CONFIG_MAINSTONE', if_true: files('mainstone.c'))
arm_ss.add(when: 'CONFIG_MICROBIT', if_true: files('microbit.c'))
arm_ss.add(when: 'CONFIG_MUSICPAL', if_true: files('musicpal.c'))
arm_ss.add(when: 'CONFIG_NETDUINO2', if_true: files('netduino2.c'))
arm_ss.add(when: 'CONFIG_NETDUINOPLUS2', if_true: files('netduinoplus2.c'))
arm_ss.add(when: 'CONFIG_OLIMEX_STM32_H405', if_true: files('olimex-stm32-h405.c'))
arm_ss.add(when: 'CONFIG_NPCM7XX', if_true: files('npcm7xx.c', 'npcm7xx_boards.c'))
arm_ss.add(when: 'CONFIG_NSERIES', if_true: files('nseries.c'))
arm_ss.add(when: 'CONFIG_SX1', if_true: files('omap_sx1.c'))
arm_ss.add(when: 'CONFIG_CHEETAH', if_true: files('palm.c'))
arm_ss.add(when: 'CONFIG_GUMSTIX', if_true: files('gumstix.c'))
arm_ss.add(when: 'CONFIG_SPITZ', if_true: files('spitz.c'))
arm_ss.add(when: 'CONFIG_Z2', if_true: files('z2.c'))
arm_ss.add(when: 'CONFIG_REALVIEW', if_true: files('realview.c'))
arm_ss.add(when: 'CONFIG_SBSA_REF', if_true: files('sbsa-ref.c'))
arm_ss.add(when: 'CONFIG_STELLARIS', if_true: files('stellaris.c'))
arm_ss.add(when: 'CONFIG_STM32VLDISCOVERY', if_true: files('stm32vldiscovery.c'))
arm_ss.add(when: 'CONFIG_COLLIE', if_true: files('collie.c'))
arm_ss.add(when: 'CONFIG_VERSATILE', if_true: files('versatilepb.c'))
arm_ss.add(when: 'CONFIG_VEXPRESS', if_true: files('vexpress.c'))
arm_ss.add(when: 'CONFIG_ZYNQ', if_true: files('xilinx_zynq.c'))
arm_ss.add(when: 'CONFIG_SABRELITE', if_true: files('sabrelite.c'))

arm_ss.add(when: 'CONFIG_ARM_V7M', if_true: files('armv7m.c'))
arm_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4210.c'))
arm_ss.add(when: 'CONFIG_PXA2XX', if_true: files('pxa2xx.c', 'pxa2xx_gpio.c', 'pxa2xx_pic.c'))
arm_ss.add(when: 'CONFIG_DIGIC', if_true: files('digic.c'))
arm_ss.add(when: 'CONFIG_OMAP', if_true: files('omap1.c', 'omap2.c'))
arm_ss.add(when: 'CONFIG_STRONGARM', if_true: files('strongarm.c'))
arm_ss.add(when: 'CONFIG_ALLWINNER_A10', if_true: files('allwinner-a10.c', 'cubieboard.c'))
arm_ss.add(when: 'CONFIG_ALLWINNER_H3', if_true: files('allwinner-h3.c', 'orangepi.c'))
arm_ss.add(when: 'CONFIG_ALLWINNER_R40', if_true: files('allwinner-r40.c', 'bananapi_m2u.c'))
arm_ss.add(when: 'CONFIG_RASPI', if_true: files('bcm2836.c', 'raspi.c'))
arm_ss.add(when: 'CONFIG_STM32F100_SOC', if_true: files('stm32f100_soc.c'))
arm_ss.add(when: 'CONFIG_STM32F205_SOC', if_true: files('stm32f205_soc.c'))
arm_ss.add(when: 'CONFIG_STM32F405_SOC', if_true: files('stm32f405_soc.c'))
arm_ss.add(when: 'CONFIG_XLNX_ZYNQMP_ARM', if_true: files('xlnx-zynqmp.c', 'xlnx-zcu102.c'))
arm_ss.add(when: 'CONFIG_XLNX_VERSAL', if_true: files('xlnx-versal.c', 'xlnx-versal-virt.c'))
arm_ss.add(when: 'CONFIG_FSL_IMX25', if_true: files('fsl-imx25.c', 'imx25_pdk.c'))
arm_ss.add(when: 'CONFIG_FSL_IMX31', if_true: files('fsl-imx31.c', 'kzm.c'))
arm_ss.add(when: 'CONFIG_FSL_IMX6', if_true: files('fsl-imx6.c'))
arm_ss.add(when: 'CONFIG_ASPEED_SOC', if_true: files(
  'aspeed.c',
  'aspeed_soc_common.c',
  'aspeed_ast2400.c',
  'aspeed_ast2600.c',
  'aspeed_ast10x0.c',
  'aspeed_eeprom.c',
  'fby35.c'))
arm_ss.add(when: 'CONFIG_MPS2', if_true: files('mps2.c'))
arm_ss.add(when: 'CONFIG_MPS2', if_true: files('mps2-tz.c'))
arm_ss.add(when: 'CONFIG_MSF2', if_true: files('msf2-soc.c'))
arm_ss.add(when: 'CONFIG_MUSCA', if_true: files('musca.c'))
arm_ss.add(when: 'CONFIG_ARMSSE', if_true: files('armsse.c'))
arm_ss.add(when: 'CONFIG_FSL_IMX7', if_true: files('fsl-imx7.c', 'mcimx7d-sabre.c'))
arm_ss.add(when: 'CONFIG_ARM_SMMUV3', if_true: files('smmuv3.c'))
arm_ss.add(when: 'CONFIG_FSL_IMX6UL', if_true: files('fsl-imx6ul.c', 'mcimx6ul-evk.c'))
arm_ss.add(when: 'CONFIG_NRF51_SOC', if_true: files('nrf51_soc.c'))
arm_ss.add(when: 'CONFIG_XEN', if_true: files('xen_arm.c'))
arm_ss.add(when: 'CONFIG_IPOD_CLASSIC_6G', if_true: files(
  'ipod_classic/ipod_classic.c',
  'ipod_classic/ipod_classic_timer.c',
  'ipod_classic/ipod_classic_clk.c',
  'ipod_classic/ipod_classic_i2c.c',
  'ipod_classic/ipod_classic_pmu.c',
  'ipod_classic/ipod_classic_ata.c',
  'ipod_classic/ipod_classic_lcd.c',
  'ipod_classic/ipod_classic_gpio.c',
  'ipod_classic/ipod_classic_disp.c',
  'ipod_classic/ipod_classic_wheel.c',
  'ipod_classic/ipod_classic_i2s.c',
  'ipod_classic/ipod_classic_audio_out.c',
  'ipod_classic/ipod_classic_cs42l55.c',
  'ipod_classic/ipod_classic_sm1.c',
  'ipod_classic/ipod_classic_sm1_codec.c',
  'ipod_classic/ipod_classic_hle_audio.c',
  'ipod_classic/ipod_classic_hle_np.c',
  'ipod_classic/ipod_classic_np_overlay.c'))
# The decoders behind the SM1 engine: upstream's code, built with its own flags
# and none of QEMU's warnings.  codecs/README says what they are.
ipod_classic_codecs = static_library('ipod_classic_codecs', files(
  'ipod_classic/codecs/minimp3/minimp3.c',
  'ipod_classic/codecs/faad2/bits.c',
  'ipod_classic/codecs/faad2/cfft.c',
  'ipod_classic/codecs/faad2/common.c',
  'ipod_classic/codecs/faad2/decoder.c',
  'ipod_classic/codecs/faad2/drc.c',
  'ipod_classic/codecs/faad2/error.c',
  'ipod_classic/codecs/faad2/filtbank.c',
  'ipod_classic/codecs/faad2/huffman.c',
  'ipod_classic/codecs/faad2/is.c',
  'ipod_classic/codecs/faad2/mdct.c',
  'ipod_classic/codecs/faad2/mp4.c',
  'ipod_classic/codecs/faad2/ms.c',
  'ipod_classic/codecs/faad2/output.c',
  'ipod_classic/codecs/faad2/pns.c',
  'ipod_classic/codecs/faad2/pulse.c',
  'ipod_classic/codecs/faad2/specrec.c',
  'ipod_classic/codecs/faad2/syntax.c',
  'ipod_classic/codecs/faad2/tns.c'),
  include_directories: include_directories('ipod_classic/codecs/faad2'),
  c_args: ['-w', '-DLC_ONLY_DECODER', '-DDISABLE_SBR', '-DHAVE_STDINT_H=1',
           '-DHAVE_STRING_H=1', '-DHAVE_STDLIB_H=1', '-DSTDC_HEADERS=1',
           '-DHAVE_LRINTF=1', '-DPACKAGE_VERSION="2.11.4"'])
arm_ss.add(when: 'CONFIG_IPOD_CLASSIC_6G', if_true:
           declare_dependency(link_with: ipod_classic_codecs))
# Only the SHA-1 block is shared with the iPod Touch 2G sources; the rest of
# that machine is unused here and needs OpenSSL's bignum code.
arm_ss.add(when: 'CONFIG_IPOD_TOUCH_SHA1', if_true: files('ipod_touch_sha1.c'))

system_ss.add(when: 'CONFIG_ARM_SMMUV3', if_true: files('smmu-common.c'))
system_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4_boards.c'))
system_ss.add(when: 'CONFIG_RASPI', if_true: files('bcm2835_peripherals.c'))
system_ss.add(when: 'CONFIG_TOSA', if_true: files('tosa.c'))

hw_arch += {'arm': arm_ss}

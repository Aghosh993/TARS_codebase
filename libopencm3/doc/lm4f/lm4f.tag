<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>doc-lm4f.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/lm4f/</path>
    <filename>doc-lm4f_8h</filename>
  </compound>
  <compound kind="file">
    <name>gpio.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/lm4f/</path>
    <filename>gpio_8c</filename>
    <includes id="gpio_8h" name="gpio.h" local="no" imported="no">libopencm3/lm4f/gpio.h</includes>
    <includes id="systemcontrol_8h" name="systemcontrol.h" local="no" imported="no">libopencm3/lm4f/systemcontrol.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_LOCK_UNLOCK_CODE</name>
      <anchorfile>group__gpio__file.html</anchorfile>
      <anchor>ga7b4058c05682f371e3c210e272a2d003</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>PCTL_AF</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>gada5265d9b7ca84756a8876b34ae5a65c</anchor>
      <arglist>(pin, af)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>PCTL_MASK</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga7af7fa835329184da3a5a53bd4ce6a72</anchor>
      <arglist>(pin)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_enable_ahb_aperture</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga973a4b2fdbe9c67f34d323e8426e45e3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_mode_setup</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga03c124c823e28cfee2bbf904ca12020b</anchor>
      <arglist>(uint32_t gpioport, enum gpio_mode mode, enum gpio_pullup pullup, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_set_output_config</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga83d7dc913be2bd27110019cae5ecf25d</anchor>
      <arglist>(uint32_t gpioport, enum gpio_output_type otype, enum gpio_drive_strength drive, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_set_af</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga08c5e974d93f738dcd80cf95b2447a6a</anchor>
      <arglist>(uint32_t gpioport, uint8_t alt_func_num, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_unlock_commit</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga070a8a3682a7e30e037a6de9b287355c</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_toggle</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>gac7fe863cd889722d5e06916617b007b1</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_configure_trigger</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga6a3a20a7f47ef2eb7c6efa9360f51fc9</anchor>
      <arglist>(uint32_t gpioport, enum gpio_trigger trigger, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_enable_interrupts</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga6647e613abe8223c2e78f773dfa279ce</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_disable_interrupts</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>gaa85cd2d1a8cbcfa7406e11afb15c1b6b</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>gpio.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/lm4f/</path>
    <filename>gpio_8h</filename>
    <includes id="memorymap_8h" name="memorymap.h" local="no" imported="no">libopencm3/lm4f/memorymap.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>GPIOA</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>gac485358099728ddae050db37924dd6b7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOB</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga68b66ac73be4c836db878a42e1fea3cd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOC</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga2dca03332d620196ba943bc2346eaa08</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOD</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga7580b1a929ea9df59725ba9c18eba6ac</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOE</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>gae04bdb5e8acc47cab1d0532e6b0d0763</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOF</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga43c3022dede7c9db7a58d3c3409dbc8d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOG</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga02a2a23a32f9b02166a8c64012842414</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOH</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>gadeacbb43ae86c879945afe98c679b285</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOJ</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga7d3020a351195b6600a5d64c01c461fa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOK</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga273d78d198f0221223b3e9d7798f1649</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOL</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga845425770e695dff46b9b28c0e7ed90b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOM</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga3ea32fef9aa376b226f5c209bb0126b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPION</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>gafaa7a6254c4bdb2bc8170b77b4813233</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOP</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>gaccc0c6189ff0a9269a991eef22b0af58</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOQ</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga30c45ca759f238ca12afe1733a309fe6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO0</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga20f88dbc839eb32b5fec903474befdd7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO1</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>gabe59d3a7ce7a18e9440bd54cae1f3fc8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO2</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga88a95401ea8409c83cbda42f31450cd0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO3</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>gaf3cc04d651b622d5323d74dc2f0999a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO4</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga98aeff9c8b3bbdfd119e4ec4d3f615c8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO5</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga1a96368c99d63b0e715b7e0421f4a209</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO6</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga46027cd97ff756e5ddadcc10811b5699</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO7</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga3820cacb614277004870fc37b33ad084</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_ALL</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga9f4da19f41fda0a3ec6d017e0bedfa4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DATA</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gadf02597783c151627da209a001fd28db</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DIR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gac3ff1d81b6ee02147458f7741b9877be</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_IS</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaf3168523aa71db397cc6b90d06c19a5e</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_IBE</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga93047ff801b7bf6f8296706703f2bdeb</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_IEV</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaea367cbbe8e1040667eef8b89533d954</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_IM</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gae19924963589e8d9ae1ce5204c92afe1</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_RIS</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaac5a97bf3ce4743eb2a15016d02f05a2</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_MIS</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga0a9247b9076a5e068506e4778bd7117c</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_ICR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga7f7bb0f9d7f416c02e635d7fae773c66</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_AFSEL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga4af3c414275112a06dd5e02ddbe94252</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DR2R</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga0386cc42ff6da71217fb156a7410fa75</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DR4R</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gad1ea20c9c60d5bc22817c71de40a7d61</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DR8R</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga3a8a81a8d969e2dddf535005b8d0d0d1</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_ODR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga58655e9db856a8f3208e388814db2de1</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PUR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga85534bcd71ffd7330699c2a19255bd8e</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PDR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga91a11047ca48e9bc9d8b4b0b72feb00e</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_SLR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gad7df547f4a985cd1ecf74f052767fd0c</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DEN</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga7eb35658e871ace9d38772762765f964</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_LOCK</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gadbed78649b4858807f28bd9f4b30cbf4</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_CR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gab033a90c9505838734359175c38ef15e</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_AMSEL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga8a312414a741e68773f02828db9e6d85</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PCTL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gae0a7b9acf9e36985689893dc8cb6e9d4</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_ADCCTL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gabfd4b002e858cb8430cd1825b71979fc</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DMACTL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga3fefaf90062772262aa8b3f2caf32406</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID4</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga043d38c836fb8896eb7e2ffa125bfead</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID5</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaf4090988152ba51816c7dd3449e741f9</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID6</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gae829c3b0903409f010f4fa995dd159a7</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID7</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga96c6d6cf6ccbb9b0d23441360c158d53</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID0</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga43ce567a69a392019261456ff8ab3dd3</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID1</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaf7dd2186df64e01fe54402b3cb0ba746</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID2</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga48b43686cc64de8d233a4c6570661889</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID3</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gad00b0b6f35df37248fb0c1dac166c035</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PCELL_ID0</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga75efbc491b08c94cf62dc2a111032c90</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PCELL_ID1</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gae1a623f6901860ed5c07ff99e66bbb79</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PCELL_ID2</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gac2510694ee55ad599b183c7b01cbd0f5</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PCELL_ID3</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gabf4b1900107c50fd8b7705a7bcc0121b</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>gpio_mode</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga620d533a2ccc5296d2f6c8b95bf89fe1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_MODE_OUTPUT</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga620d533a2ccc5296d2f6c8b95bf89fe1aa5824cd687ac84449b093087c25cec7c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_MODE_INPUT</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga620d533a2ccc5296d2f6c8b95bf89fe1a61ddb59c5e9a9cc6d9cf99d11a36920b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_MODE_ANALOG</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga620d533a2ccc5296d2f6c8b95bf89fe1a5a5b8916bf45f2a4ff3818fa75f85d27</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>gpio_pullup</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga45e6c8e8730474e4644929cc5528400b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_PUPD_NONE</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga45e6c8e8730474e4644929cc5528400ba7807e2b9a6941fdb1326d9506af81a67</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_PUPD_PULLUP</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga45e6c8e8730474e4644929cc5528400ba10489125ed48a34dd001fa33e13ae396</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_PUPD_PULLDOWN</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga45e6c8e8730474e4644929cc5528400ba700268e334e769a26b44a8bf253017d2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>gpio_output_type</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gad59707c5e578193702cfb29b49b88108</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_OTYPE_PP</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggad59707c5e578193702cfb29b49b88108a6f3629cecfb427731be8487dce2970ef</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_OTYPE_OD</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggad59707c5e578193702cfb29b49b88108a2f6c5d8dcae60adb9f33fa337fad91ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>gpio_drive_strength</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga14eba84c1c8f80b08a770775d3bf060a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_DRIVE_2MA</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga14eba84c1c8f80b08a770775d3bf060aaea65a1945ac5ef1b9c311ce0795a136d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_DRIVE_4MA</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga14eba84c1c8f80b08a770775d3bf060aafea3985b235eba4f68685de32f0fd7a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_DRIVE_8MA</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga14eba84c1c8f80b08a770775d3bf060aa769a972b3ebbb6ad8dcf54abdb24df10</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_DRIVE_8MA_SLEW_CTL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga14eba84c1c8f80b08a770775d3bf060aa13cff51bcb9a3b105a9d0c288bf73b83</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>gpio_trigger</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaf59a0ba6300bcfc24d4dbf13000a2fbe</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_TRIG_LVL_LOW</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggaf59a0ba6300bcfc24d4dbf13000a2fbeac94012614bbe9b2145034b62207cc52e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_TRIG_LVL_HIGH</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggaf59a0ba6300bcfc24d4dbf13000a2fbead8f2e505b5428ce834d192e911b87265</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_TRIG_EDGE_FALL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggaf59a0ba6300bcfc24d4dbf13000a2fbea36956d1dafc73d361288ad3349b5ed19</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_TRIG_EDGE_RISE</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggaf59a0ba6300bcfc24d4dbf13000a2fbeae7e4da82de08018b2a2283404ad0bb91</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_TRIG_EDGE_BOTH</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggaf59a0ba6300bcfc24d4dbf13000a2fbea001639d96fae849f0c4672b265237da1</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_enable_ahb_aperture</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga973a4b2fdbe9c67f34d323e8426e45e3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_mode_setup</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga03c124c823e28cfee2bbf904ca12020b</anchor>
      <arglist>(uint32_t gpioport, enum gpio_mode mode, enum gpio_pullup pullup, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_set_output_config</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga83d7dc913be2bd27110019cae5ecf25d</anchor>
      <arglist>(uint32_t gpioport, enum gpio_output_type otype, enum gpio_drive_strength drive, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_set_af</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga08c5e974d93f738dcd80cf95b2447a6a</anchor>
      <arglist>(uint32_t gpioport, uint8_t alt_func_num, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_toggle</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gac7fe863cd889722d5e06916617b007b1</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_unlock_commit</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga070a8a3682a7e30e037a6de9b287355c</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_configure_trigger</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga6a3a20a7f47ef2eb7c6efa9360f51fc9</anchor>
      <arglist>(uint32_t gpioport, enum gpio_trigger trigger, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_enable_interrupts</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga6647e613abe8223c2e78f773dfa279ce</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_disable_interrupts</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaa85cd2d1a8cbcfa7406e11afb15c1b6b</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint8_t</type>
      <name>gpio_read</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga4efb268c94f062228de2339d8273af82</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_write</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga86b48f47f23889ffed56ab86c0605865</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios, uint8_t data)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_set</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga8ce0832cd5110fa7bbe0f6c968a04a09</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_clear</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>gaed9a0283ed78d8746f72a298889207f4</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint8_t</type>
      <name>gpio_port_read</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>gae3a2f79057d2a87a5f08b28242ea8ba8</anchor>
      <arglist>(uint32_t gpioport)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_port_write</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga00eeea592868471ce4eecce176b062f7</anchor>
      <arglist>(uint32_t gpioport, uint8_t data)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>gpio_is_interrupt_source</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga6526582457c0936b5a978b2548336610</anchor>
      <arglist>(uint32_t gpioport, uint8_t srcpins)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_clear_interrupt_flag</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga0eff5e23114c6346aff26c0b275d7464</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint8_t</type>
      <name>gpio_read</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga4efb268c94f062228de2339d8273af82</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_write</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga86b48f47f23889ffed56ab86c0605865</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios, uint8_t data)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_set</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga8ce0832cd5110fa7bbe0f6c968a04a09</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_clear</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>gaed9a0283ed78d8746f72a298889207f4</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint8_t</type>
      <name>gpio_port_read</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>gae3a2f79057d2a87a5f08b28242ea8ba8</anchor>
      <arglist>(uint32_t gpioport)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_port_write</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga00eeea592868471ce4eecce176b062f7</anchor>
      <arglist>(uint32_t gpioport, uint8_t data)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>gpio_is_interrupt_source</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga6526582457c0936b5a978b2548336610</anchor>
      <arglist>(uint32_t gpioport, uint8_t srcpins)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_clear_interrupt_flag</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga0eff5e23114c6346aff26c0b275d7464</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>license.dox</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/</path>
    <filename>license_8dox</filename>
  </compound>
  <compound kind="file">
    <name>memorymap.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/lm4f/</path>
    <filename>memorymap_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>GPIOA_APB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a4f916d7054a9ec3e15a52c9ee5cdcd41</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOB_APB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a85365a2040afb8c4c43bc1f113fc69f3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOC_APB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a58014fad55bb230484f31b3618ec9ef3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOD_APB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a82a89c28f10b343529fadb20bc36353f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOE_APB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a066a72e80697521c7d66c213c0dd2129</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOF_APB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>ae537ba38be59eb9372b3509c4132d4e0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOG_APB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a514982b973830d61192b86655c77cd3a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOH_APB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a8ebcb01a3bd2f9af12034bce09ec3829</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOJ_APB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a00640e760efab416419a4786dda55e14</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOA_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>ad7723846cc5db8e43a44d78cf21f6efa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>ac944a89eb789000ece920c0f89cb6a68</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOC_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a26f267dc35338eef219544c51f1e6b3f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOD_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a1a93ab27129f04064089616910c296ec</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOE_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>ab487b1983d936c4fee3e9e88b95aad9d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOF_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a7f9a3f4223a1a784af464a114978d26e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOG_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a5d8ca4020f2e8c00bde974e8e7c13cfe</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOH_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>aee4716389f3a1c727495375b76645608</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOJ_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a73f5a4e42f41acc614ee82c8ebfe0b85</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOK_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a46d3f8cd7c045b5e13cd7395b8e936e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOL_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a8f574b9654493900fb5d96c16d283199</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOM_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>abecd984be43c5b0052ded8c47c034c6c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPION_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>aaa504fafd3506c282580dfa94479b1c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOP_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a7d837880331497152f3c8200e25e887b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOQ_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a2e5b5bb3cc8e0826de57f8fa79d63f4e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART0_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a7a07348b4332ff6b88abf6092347deba</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART1_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a383bf0c4670c3a7fa72df80f66331a46</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART2_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>ac9998d643534960b684d45a60b998421</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART3_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a2eff3896840fdf741bd67d2d7fe99a34</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART4_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a94d92270bf587ccdc3a37a5bb5d20467</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART5_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>aa155689c0e206e6994951dc3cf31052a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART6_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a77f0753cf94e8b7b3e04a52a896f9dcb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART7_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a3150e4b10ec876c0b20f22de12a8fa40</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI0_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>ad4d3af97208d1f383e6fc55b2cf22274</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI1_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a1d1c681e3321c27f2de198ed8ca2ce95</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI2_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>ab9400dc20cd522bd8505f7c2304b68b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI3_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>aacf71783340df06b2a70d58d7f380530</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>aa6c4cbed4ddbb3ecd77de93fab2a2e04</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a568834119b9aae01b7f69e8a012d3bf9</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>nvic.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/lm4f/</path>
    <filename>nvic_8h</filename>
  </compound>
  <compound kind="file">
    <name>rcc.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/lm4f/</path>
    <filename>rcc_8c</filename>
    <includes id="rcc_8h" name="rcc.h" local="no" imported="no">libopencm3/lm4f/rcc.h</includes>
    <member kind="function">
      <type>void</type>
      <name>rcc_configure_xtal</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga2cd8f194ad903834c78212a0eeb05aa4</anchor>
      <arglist>(enum xtal_t xtal)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_disable_main_osc</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga5f536734177d63215d1494e6dea715d0</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_disable_interal_osc</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga25df4f03d0154ac5b09b875dad1226e0</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_main_osc</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gafc653bc7b616053515e1fd4aeb54f972</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_interal_osc</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga191546fa4ded6cb6d3c753e0de255464</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_rcc2</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gae166a674c8b6592adea0ff77c1e009e9</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_off</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga31217830e7f538e902e8b157e2715428</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_on</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gaf58b85261f65604132b67d3567c2b0c3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_osc_source</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga6acaa4f857ca454c3f90f579091b3246</anchor>
      <arglist>(enum osc_src src)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_bypass_disable</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gabbbe68ef690e48ae19d4ff04e69cc9c3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_bypass_enable</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga269054cc63981ae593e6820de2fe76b1</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_pll_divisor</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga104041e1468a0fc62d6c584a25859053</anchor>
      <arglist>(uint8_t div400)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_pwm_divisor</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gad9dea6bb566a5d6079037f41cc3d1c62</anchor>
      <arglist>(enum pwm_clkdiv div)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_usb_pll_off</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga68c7027c77c2a1a0d6d202e191c1baf5</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_usb_pll_on</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gae31973474f6d00125a57784e84230c86</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_wait_for_pll_ready</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga0f5f3bea5dbfde10760a9cd775951a51</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_change_pll_divisor</name>
      <anchorfile>group__rcc__high__level.html</anchorfile>
      <anchor>ga1c5a8dbbc0a6bac380b0041962075269</anchor>
      <arglist>(uint8_t pll_div400)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>rcc_get_system_clock_frequency</name>
      <anchorfile>group__rcc__high__level.html</anchorfile>
      <anchor>ga14ed842866228082443acad62da172f4</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint32_t</type>
      <name>xtal_to_freq</name>
      <anchorfile>group__rcc__high__level.html</anchorfile>
      <anchor>ga43a00aa2241426ed96573abc3de839e9</anchor>
      <arglist>(enum xtal_t xtal)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_sysclk_config</name>
      <anchorfile>group__rcc__high__level.html</anchorfile>
      <anchor>ga94e682bdf26bde662afb76403acdd227</anchor>
      <arglist>(enum osc_src src, enum xtal_t xtal, uint8_t pll_div400)</arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>lm4f_rcc_sysclk_freq</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga53588dcb636da3e059dd7c45d82f6504</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>rcc.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/lm4f/</path>
    <filename>rcc_8h</filename>
    <includes id="systemcontrol_8h" name="systemcontrol.h" local="no" imported="no">libopencm3/lm4f/systemcontrol.h</includes>
    <member kind="enumeration">
      <type></type>
      <name>osc_src</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga1b534c93284a244cc1a84f41a459173a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>OSCSRC_MOSC</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga1b534c93284a244cc1a84f41a459173aadb9b675853b16e89501a2eb677c12894</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>OSCSRC_PIOSC</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga1b534c93284a244cc1a84f41a459173aa7168eb777903d908b7d91b952b3f1c58</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>OSCSRC_PIOSC_D4</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga1b534c93284a244cc1a84f41a459173aa24ecedc2dea27d6b5823bcce0133fa22</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>OSCSRC_30K_INT</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga1b534c93284a244cc1a84f41a459173aae39fab718557fa25a344a38c6d8469f3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>OSCSRC_32K_EXT</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga1b534c93284a244cc1a84f41a459173aac0bd6d9d0b5e0de9831d717589345c11</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>pwm_clkdiv</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga9634afd06a58256ecd18d15d3fed8e5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_2</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5ca15284be2c0bdfb7cfd1abc33586a0bce</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_4</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5ca192482b9872c838aa8439a7dc24872f3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_8</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5ca2b247289718884cea03bbf09cc6caf7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_16</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5ca5a9382f819f076e61ac36cbcb10f756c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_32</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5caf4549a3497ee484e5a6a2fc41a4cdb29</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_64</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5cab881e4379f382748ece5ec8fd5252b3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>xtal_t</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga8f09204cad649f070887682ed6549ea9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_4M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9aea78bea9d38f290229c93b114486dec9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_4M_096</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9af5a8c78ec3fa62fe9db636244d7aae2d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_4M_9152</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9aa9365b14fa1c2d02f3c03d644ecc7c5d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_5M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9ab0fce32c50d64cbd11cdeabe3c803a25</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_5M_12</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a06584b157ab517bfec8d48aa40d6134c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_6M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9aeb2a3ddd53e05557c3ffcea38adec4fe</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_6M_144</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9afcfaf7f8ca0ce1bc94fbb15e05d9c2cd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_7M_3728</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a0d4bc0fd92bd1e48594f5f19a3d38d3c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_8M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a76156a0243858d32154f20628fe06ea7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_8M_192</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a206a359c0b7effe7044e2036329b522c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_10M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9ac5ae1bca5c59cb91d93b693d123e7507</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_12M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a9ffa2eea00a874b497e48f28c9760920</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_12M_288</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9abb413dfd67aec6c42a74a4344371dedc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_13M_56</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a4364c57b346dfab730f7d5a8fc481343</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_14M_31818</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a4e82b3d2c5a44644ce20873733cabbfc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_16M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a72d6d58d8b24db9d249f76876e753016</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_16M_384</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a0712bb6a91be8e9bba76a804a267139b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_18M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a43224b6f62384fbdd777e598a2013a36</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_20M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a78ec47d31d8759b95ba7b939919ea68e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_24M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9abc98d766583e32076c0504b9297dfa8e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_25M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a8ac164b085138f1c24abb51ee35cf5d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_configure_xtal</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga2cd8f194ad903834c78212a0eeb05aa4</anchor>
      <arglist>(enum xtal_t xtal)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_disable_main_osc</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga5f536734177d63215d1494e6dea715d0</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_disable_interal_osc</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga25df4f03d0154ac5b09b875dad1226e0</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_main_osc</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gafc653bc7b616053515e1fd4aeb54f972</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_interal_osc</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga191546fa4ded6cb6d3c753e0de255464</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_rcc2</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gae166a674c8b6592adea0ff77c1e009e9</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_off</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga31217830e7f538e902e8b157e2715428</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_on</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gaf58b85261f65604132b67d3567c2b0c3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_osc_source</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga6acaa4f857ca454c3f90f579091b3246</anchor>
      <arglist>(enum osc_src src)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_bypass_disable</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gabbbe68ef690e48ae19d4ff04e69cc9c3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_bypass_enable</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga269054cc63981ae593e6820de2fe76b1</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_pll_divisor</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga104041e1468a0fc62d6c584a25859053</anchor>
      <arglist>(uint8_t div400)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_pwm_divisor</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gad9dea6bb566a5d6079037f41cc3d1c62</anchor>
      <arglist>(enum pwm_clkdiv div)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_usb_pll_off</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga68c7027c77c2a1a0d6d202e191c1baf5</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_usb_pll_on</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gae31973474f6d00125a57784e84230c86</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_wait_for_pll_ready</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga0f5f3bea5dbfde10760a9cd775951a51</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_change_pll_divisor</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga70fca8e561f7e2b2b7062c22d85419b4</anchor>
      <arglist>(uint8_t plldiv400)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>rcc_get_system_clock_frequency</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga14ed842866228082443acad62da172f4</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_sysclk_config</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga94e682bdf26bde662afb76403acdd227</anchor>
      <arglist>(enum osc_src src, enum xtal_t xtal, uint8_t pll_div400)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ssi.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/lm4f/</path>
    <filename>ssi_8h</filename>
    <includes id="memorymap_8h" name="memorymap.h" local="no" imported="no">libopencm3/lm4f/memorymap.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>SSI0</name>
      <anchorfile>group__ssi__base.html</anchorfile>
      <anchor>gac8ab5d11c2608b24b105c74da01088f4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI1</name>
      <anchorfile>group__ssi__base.html</anchorfile>
      <anchor>ga26e99cb407c039d9b5fa260e3a0c463a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI2</name>
      <anchorfile>group__ssi__base.html</anchorfile>
      <anchor>gaf7a5b340fc1e5948b228e5d13a2f0181</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI3</name>
      <anchorfile>group__ssi__base.html</anchorfile>
      <anchor>ga7fff594b0952c3f9ae26e62893661cae</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_CR0</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga42848fe0c117f1ff719f3801d5aa2680</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_CR1</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga10464498ce11115ad90d7db3a9b343e1</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_DR</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>gaca41a444d99801dc07b1609434322289</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_SR</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga6dbaabaf26a3cf9f799e2aedda64a976</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_CPSR</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga6d711191bee46cd63ac6e008bbe89415</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_IM</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga36b5b43d0230a804949f729ff596c9f0</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_RIS</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>gacd096e24500cffbcaa3a7a80d5d38b29</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_MIS</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>gadeeb5ebb23eb805e39b7861f4c355d8d</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_ICR</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga70464021aa00aa847de064a816b4d9be</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_DMACTL</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga8c163597675e79c701f72032dfc2193e</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_CC</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga96f69900e8c3887c7dc82d831b5e8205</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID4</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga96471d945829658d9e03fb90ab43c937</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID5</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga5d39ef31f481f2df24abe6dc5e865c3c</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID6</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga51b58160ceb9ffc45d614313939d3e6d</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID7</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga063c1d8b96e1f8f71d6892d2a362a947</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID0</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga239887a62ecac0478facf58f7b2915f4</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID1</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga559c77aff03678ea9b88be82ba02d24c</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID2</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga6ad225fcc9ce118ca061cfd88cbfcade</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID3</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga3be5ed28af28047f9df642d3020a1d03</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PCELL_ID0</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga92d02e0dbe52ba455b6f374d29a4d866</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PCELL_ID1</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>gac636b0d8ffdb95f3d285a8e82c976201</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PCELL_ID2</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga22c47d6c4a1316204db952454b270634</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PCELL_ID3</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>gac92b2290be2869027821a1fc31c5a43e</anchor>
      <arglist>(port)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>systemcontrol.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/lm4f/</path>
    <filename>systemcontrol_8c</filename>
    <includes id="systemcontrol_8h" name="systemcontrol.h" local="no" imported="no">libopencm3/lm4f/systemcontrol.h</includes>
    <member kind="function">
      <type>void</type>
      <name>periph_clock_enable</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga82ac341aee840ff3b2fc15d79de016c9</anchor>
      <arglist>(enum lm4f_clken periph)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>periph_clock_disable</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa8ca86965f943013fd3bc6eb61f889e2</anchor>
      <arglist>(enum lm4f_clken periph)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>systemcontrol.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/lm4f/</path>
    <filename>systemcontrol_8h</filename>
    <includes id="memorymap_8h" name="memorymap.h" local="no" imported="no">libopencm3/lm4f/memorymap.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5edf22a6ff060d03941c304b59f0356c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae38a424f0c881dab2d160d1e0a9a6131</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PBORCTL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga76a268a55ee06d4523cde5e981e25d00</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_LDORCTL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae1727f4e9282812be6f932cce51ddf65</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1c81f0f10746ec3948b6f46b4d06583f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga846407f0f364f2507891553773c77d19</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4f9f87f9381b3da165a82bde242d5e61</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab6b8c95db38367853b060f1c7fa53121</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga23e27fd6ce8421975c6d703068dc8fea</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLCFG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gafd79d5907c1916a809d7f7a4c840fd1e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4c4d9a120e8f86cd11fcf8e2af7b3104</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6a83bd485e529b5a78559157d5569755</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MOSCCTL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga25e07d3c958f6bac10d5a290d57bfae8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DSLPCLKCFG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga68c7187602ac287437278b7680d80ed1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SYSPROP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6daeba541534e298f90efd0f35b8b5c8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCCAL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga250323d022f46409e1d0f58a3e953f8a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCSTAT</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaccae2718c49e898e670929411106fddc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaafe0fae5b7bc00e01652773e2cb03d38</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga734cc69a16872dc5b0435dfc8e33ce8c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLSTAT</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga22dba56588d81f1db2a533a89128eb89</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1efbf63d9e8aff1b23bce262faef97db</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaadcd15bb3c76427ca5cc1527fc986c15</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3955755f8969bdb31e8a6120e3858947</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga894da3bc3acb2228e6885633818452b1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3c4439eec911da39692e2eca91ad133e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaafabd906ed5f891d6c2850ac476a8855</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga9d6cd0bb2d9ce4370183ccdf3a0d0f29</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4b3c3f57adcf2c1ad7078338ba1917bf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga9b4d83e2f9614143848a83cb2e98d45e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga2ed5cfe445620462380d37ae203af89b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga33d54fad389634e98f72b7d0c08aea27</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6628dce9c865dbd404cc128cfc4118b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae7f36b9ab7e2fd3407fc126a4de3cccb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1a632d7f025208b96eb7f076792f4b69</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPEEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabeabdb669aa33e8b4d567f04f6881739</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga11ffaae252e1fcdfe3b46d4b52069f97</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae52f07e6026cdffcfbb66c7e949254d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0c76a8ca571ad439072226ae778731d2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga2c0070136c26a60db90b69c8a36d55f1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga85fc6d04e544c769ef810f4b20335e62</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabc341b62d00a679132decfb16b08abb2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad1cc563458c06d54fd6ad1a005f5136d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga8957ae37f65450d5eeb330dbba058fbe</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gacb38bc82810ceb53602fbd9315d600bf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf6c69bc3b36566ab6b64e00bff1b2c8a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae0cf4514b7157ff3581048b8f4f7afae</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga96f2718fb61767be5ce86c5156b4be76</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga9cc6e115d1a61a43e522f8ee98c8cd42</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga9c75e336111c2f7be84e3d946070eaf5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga87306cd6fd517cdff582c1ac03317e06</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SREEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga39d9833a54e27edd54148e774f7f8140</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4e0f2b7801661f4b53ac3318ba0558be</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7fb6c282f04b8c0049a9a37b3c31ffde</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac2a17a27f27095d9183b01b62f4b6595</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga8af77153922a20e34a61458fe52a01a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga63b94e1ee057d847a3b743d2225b30c8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad18dd4000352df15f7747be4b2b26dd1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad6dab476e4bdc0c7a076aeb076824c7a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad9ab60781683a7d1034bc15d6580be9d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa89f93f8b0550662977d2042ca3763a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga2b4a22e34f60aa16590503bd28142e93</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5887eadb4604a6652da75d2da21704fe</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga370831e771886d3f1f55032fc3f68ac4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga939c17fa3fd86b64ff052438913e1852</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab690911dbceaab7546c4e29285215005</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga475582de09fc682400a2851e9487f84c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCEEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadb6f19b04e5d634d7b77050c9974cd00</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gace339e2ae7d3ef98917903d02f56867d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gafececbf0e6767bb73907652f4fe917b1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga21e1a44ea9d264cb32e3635022a2df7e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaebf3e2e2e2282562e8a5db6e2523f4a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae538e8bc54cd976ce5fc19175077a417</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac5c80a3adce209fa73b22b617dc3c4ef</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab775e111e8f836f33fe305991d248a09</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab153bbf9505acdf02e6edb0a7c628370</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga32248274bb68b0db5373fd906c87e1f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6968f26f5c99c0bae49a8dc8cab49c26</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gacb97dbf5e9f92b919853903369364329</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa3c720bebe058ac6cc7509f01b6af037</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga747164418de6ef193383a654d6928a21</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gacccb6cd8b29fbbbc4b992d9b0cbabdc5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf0cc1ea315d6f1d229c44995d513098b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCEEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7276e913a291bc59d4fe4567df87374e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0e8aee59e053bd372ce3489621d7b2e0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3feb68760536b91d6692fb33f852b0a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga2d041d9c90f92c684c4a94c671ae6a7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga28485c0da92d2cfe63d28828acc9d738</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae95030f6b3efd410b411ac692b91b125</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae6881bf890b30baf274a7612f8bc6ef5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7a752c2bea315fb46115e1e26ccd01a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabb9d58b230ec70a87d1cce3bb4881e34</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0e80aa32ef1f2784611f99e0272d47f0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga40dad8c00b7c7da2edc386fb22cef20a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabe1b1a21bc6e8d51d14720473baa7498</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab5ac513e510f39d285a9450e3edb3036</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga755efcbb2b5fbfbbec6a7b0bd0f617e0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga161b9a5692a946c4e523d0e2af5e3768</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga33ce68cc9b4d5d1166336e6ef3399529</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCEEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga280dc3868cf374a65e7e20d5a94d6cda</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6b22c80cb897cc0217e0315e7d555475</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga900c50df903fd51991022c9cc9f7076c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab858a6231af82b2258787ef5cfe53732</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf1bed14a3a8eed561aea3e5c84c40b9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0837b1f4fab2e15dd731cf2d1606c498</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac07b587d02a1ace8b61991a6f30842a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac949c8408d49d794e49ba2b16441b7f1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3ec3fed2ee8b70aa37db0ae2841aa8d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae4c8cb028475676d356e3ef9a07dcdc3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae70bb6bf8e20657594f5b6373e193a55</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab48eb50560f5b6b458be977397714f49</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaba3f1792eb395a0f8b38d4c961e2a59d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa8ba78c3d6d3a656e5a138c4d5db056e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac5755edacae296f8e5d4627e1315a057</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaba3ff5073c7fc9d81a5ffdbe432efd41</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PREEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga72036e113c08ca9ae659b53783fd05ee</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6486d5d92ea650b07334969a7ab5841f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID0_VER_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga16fab30229e50e97c454779527ff21fb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID0_CLASS_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf8bb02a59247d7b08c480087ab2e1480</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID0_MAJOR_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga668bea5191a35cf5ba22de0b725db302</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID0_MAJOR_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga668bea5191a35cf5ba22de0b725db302</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_VER_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadf0fa38d20bea1cf67cc0afda5a1f633</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_FAM_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga66b84de779d7b3dbdb4554b4f9dd60b0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PARTNO_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga75bb789203b3904316be5bbe9b135ffa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga116bad021d6f0688c725d02fe0f59e09</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_28P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadec1e38c0b22d94e2ee49ea99bc9ba0a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_48P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga943ae94331b490333036170ab4543efa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_100P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa586a1208e6ebcb6a4ff98d94c187779</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_64P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga918446ea5313f4a9e416b7879df560fd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_144P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga946f0fc4a6c5d065fa07d5250379ae4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_157P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga63a97e645822b8f4eefffd9e26c7ad69</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_TEMP_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaefcdc20b79db7ea39dca801de7d05bcc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_TEMP_0_70</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaeb5fe6056323d0a8c56139245cd26a43</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_TEMP_M40_85</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7773b3919bfdf6277a09b685e45f361c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_TEMP_M40_105</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac4dd7e8b70faa2e9665be17be9f09353</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PKG_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga9c8296b6274230daca4a92527594e967</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PKG_SOIC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab126cf34cf0698a1bcd5045438b1a9fc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PKG_LQFP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga213d722ec1c0fa19739e73e2ccf74b34</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PKG_BGA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1883c4b05238910aebe405d2b58a3db0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_ROHS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad0ae95623f9e30aea72f842faf4be39e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_QUAL_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga29a41f5c7cec39ba357c62b73e2aa3cd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PBORCTL_BORIOR</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga17be2a80a5f12b1f3adb947808fc9a53</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS_MOSCPUPRIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab0bfa3bae579b53620e5c32eeea9754c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS_USBPLLLRIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gafbff7e2f4937563ef5396053afc6713a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS_PLLLRIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab4b46aa0c168f00095a5a4994467c539</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS_MOFRIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga043a18036c8a16fa3c72fc2823af8a5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS_BORRIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga8830434fff930be690343b1a506c5802</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC_MOSCPUPIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0e9e13a620e765e7736850ab7679eefc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC_USBPLLLIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga18f7c58233d8b834658aa8a7588b1316</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC_PLLLIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf73977c87d091aa6159296f472745bf1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC_MOFIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga54fa62dfa94ad6fe4cf33584b92b1840</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC_BORIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1e2b48e339478299a6d07294ca3979bf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC_MOSCPUPMIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga39b53fa64655b23d0a0fdbf9f604dd46</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC_USBPLLLMIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0d75024828b61b8570be7a518d0d308a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC_PLLLMIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga393020f45f93446f69823444302de1b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC_MOFMIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1cd11060075cd940baa520848f9092d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC_BORMIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab3ca5b7ac46ab12145d4f7f2d29cc0bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_MOSCFAIL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga78a489a3461f0a1030166f2eb457b01a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_WDT1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5259680ab0eede32935d6b6ee12744f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_SW</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae8083764645fb2f1c454121a9cd6c280</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_WDT0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae5ef0182c55f62f02947303b53f6faea</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_BOR</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga09aa727dc2aff547d05a0acd47b2887a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_POR</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3e84c488a026bab3831a04c55f3b7f27</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_EXT</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4ac589004611f905c614ba65329f91ca</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_ACG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5590c014f49535187c545a5ab327e6a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_SYSDIV_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5ba712f7625b543eb4888ded74a4a869</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_USESYSDIV</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf8fb495f4cdd46774964a5f822087001</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_USEPWMDIV</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga10e7f6f0d034bae2138721e90fc9dd6b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5e15a75863a2b288387662de01f59e84</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga01f48838c712ddf4b86b76682258e3bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6ebed8830a1a8a604eca53cff3bc4ef8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_8</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga129ecea0a47ab0c30f54f004b154db07</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_16</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad4fd684b00e4409765dbd296762536d7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_32</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga20916fc545dbab9a4ad2a8a9b16578d2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_64</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4f38be712c8610d138b05e8e92b56520</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWRDN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga35905e3ba9779f862e9e0e6c33061158</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_BYPASS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga97ff3e281b18a61b1d23718154d69110</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1e3c5cee1ded72a0214bf73697b6cea2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_4M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad4c98ea4884c22d748c5e8c4f5c78d93</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_4M_096</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga8b539118e530ad56b93d999b7bfbfb38</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_4M_9152</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga18b7aaadd8cba93246074510129219a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_5M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7448a6a094aaef1fd7f75728ede64b52</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_5M_12</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gacdd0dcc9611ee726742f9da5036cff5d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_6M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0062596e239b082956255a032b32e91c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_6M_144</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga11b18c90f9ff662fcff003375adb4936</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_7M_3728</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac3231b318f4c4f9ea6838eee6d9df126</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_8M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga40fe004c9131158bd830593432d88041</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_8M_192</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadfec5bed87d78fdf699e58f22ceef265</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_10M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga84a40c104aefdb2756f3e120bb2ec874</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_12M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1f0e9a28a8eb6e8190d0f62bc0ce5179</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_12M_288</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac9f64c7d381dca626345e272e922c89d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_13M_56</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gafdef4b66c2f1d81cae7face37596ddcf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_14M_31818</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga10e2c183d4ac23267ffe080f3b485082</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_16M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabbe8f1289943564b54d1edb99387406d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_16M_384</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7fba322dac3a69e304e11f46ed8bfbcf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_18M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae27885cd48447c40c66e41b9df8e123e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_20M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaaab9e55393bbacbca286a367c60037d9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_24M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5ab501f6af5ec4d42c884c7608a3e37c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_25M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga014129309ae8bfea4cb997ad49b85fff</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_OSCSRC_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadaba83b7fd53775cb6545334f638afb3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_OSCSRC_MOSC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad28d62e26fce970839bcc6157cd35440</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_OSCSRC_PIOSC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5277826f37c52e1ffa794139cb747eb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_OSCSRC_PIOSC_D4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf9cf675e83698fb7c04947091811427d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_OSCSRC_30K</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf1861fb0e7919a84216ae2950d4f719a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_IOSCDIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaabd0bfee458c4b7f2f483ec08fa9c82d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_MOSCDIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3d2bb3a1bc4e46017c5c8f25185e3ed3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTQ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaebc0ec7e19ad5accb7fe77d661016799</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6397fcdd0d640d0ebfeed87686903eaf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga65d9811ffe09003d97d0e539dc45902f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf35a041a06e9f9a93d89226121723bf6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabc26d305bf549408317368f84465d181</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4b0379eca5a5a92d72acc564aa3e2e00</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTJ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadbd5f1e4f6e99dbbe0c659fcb6011dc9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTH</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga25493559f5191ab7e90a452be6a0d5c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga621b3c71764bcb53c53110e222f7e9ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTF</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0483090ce2140b5db668459ae294d12e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTE</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga17d2d81d71284849b077d6e446ebc06a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6c2eda9f1a6327b6a8231658c669bda7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad23f76581152dc78675508920b334e52</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga30e677036aef32db26aae7297b732baa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gacbf33723e49cff0a0748381de5933880</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_USERCC2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae5f20734a9ca83207a16e5ef75318940</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_DIV400</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa06ac42bcfd26bc1d713eba6f7acf74d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_ACG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga09f717daf244a1efff38b95a2eef83bf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_SYSDIV2_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6de1a25f37d60bb4f2f49a4b691a6f83</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_SYSDIV2LSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga22f13ac333a74a868044424d01f853b3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_SYSDIV400_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga89b6d9fb176ef85e127497218a3a91bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_USBPWRDN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga27e58030fca66cf089e0fd0229c0db28</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_PWRDN2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5aeb661402986a55ada899546102f7df</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_BYPASS2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga00ba5e9742453395621589a080457f63</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gafb523b9664fb8539080b41106d194a89</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_MOSC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3a0072b73139dcfa4e086e4e3d8aca16</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_PIOSC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5c6ce2235aef567e8ea9feda8a8996e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_PIOSC_D4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7be4e4e29831cd4fbf146d72ee84fa39</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_30K</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6e1303766e78da4401a59ab99daec0ec</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_32K768</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaec6a1781103cdfe45b390d6856f75a97</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MOSCCTL_NOXTAL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaad290593a7a5690ac2f0515a1ef83cb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MOSCCTL_MOSCIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa7a5cb84236f5c8c176ed467c578dbd2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MOSCCTL_CVAL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga092e760a4bc66d3544f29ff175df9366</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SYSPROP_FPU</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga23fe88bea7d69df255daedf476353743</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCCAL_UTEN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabe8883f93985822cf049113037a37158</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCCAL_CAL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4b7490f14b79ecf8591ed125410c3b91</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCCAL_UPDATE</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5cae7d8761f0d7acde6a1a5ca285d3e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCCAL_UT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab0fe6557190dc884a37bc89884e122d6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCSTAT_DT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4a35717735468af3790f3a4526cdc2c1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCSTAT_RESULT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa44363e26b73e3ad79a09860e812505f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCSTAT_CT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5c01b10085ffc992f64dc59512ea6d7e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ0_MFRAC_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae2e13fc92489891e24595b8cdb2231de</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ0_MINT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3e441aca630218f74f104b5039180335</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ1_Q_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6db7f0f8f99685158a3678015c839a92</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ1_N_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga19fb32e61c40610b8c9beb55ed808946</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLSTAT_LOCK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3b453f1ca6c5b4cb11e25d7639fb7999</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>lm4f_clken</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga216f3c694b4491151311f37062d17d7c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WD0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca4ed4b9759029811b3e6649cd52b061e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WD1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3da71e1dd5031a1ef729ae46b721442a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa6ee7be0a28b7b0863ffa3036d6348d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca41d26b0f1d712fd1b5e4907ec6c003bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca0cbcdf4908b061573cc0c4755c7a63cf</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8cd23ab393b1e48df121992f01aeda8b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca2ebf4a17f114d0755d56c70270296856</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca742bb71633532784defb27bc08929131</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3679ad09af8988bc42fbb91d3f4dc680</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cabe0e8ee9db293583511579bffd694b6c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caca99d40a98c0cccaa8dd4a5eb8422dea</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cade83c8caa918ccda0766791076460d1e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOE</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caecccb0767e9ec411349051aa5f055779</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOF</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac2a10a31a77eac6a9e5ef9b59397abf4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5db55f45d23bd84c2e4b9a36c7cdb57e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOH</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca600bd6f61da0eaed2c2f8fdf49fbb9c9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOJ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca102cef7c361f0abf6978c4f8d0dc9fdf</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca4d57a22df2e2953d15d9d6cf17452368</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca348b6ea39fc7fa98a87655fdc6a502c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa58410e8cbf0f1d4225ee31127bd3cc2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPION</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac669e50f84467592773b50cb4939b1db</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caded2683ae4d5dc85648ce717abed5b4f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOQ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca58f20fdf421eb45995d87dcfbacd7167</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_DMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3ae8e2842eb2835ad420db68142cb4d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_HIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cacadf3729d36aff283a6b995c50d07a9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa9acb07259a107f34fb9bc1465318358</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5114880810642497fb718a780cb97199</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7e1da0d80c6c33928981ad95122fbb4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab8b4af332d6573bfa11f678c012a6a30</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cacddaf98763a8dc6edf0b69c983becfd7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca0d7a994107ab9af33f9bdbda74ec4685</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART6</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac1d0755185366f087db203361eee488c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART7</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab404d28f5b889694df6aacd5c5ee204b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_SSI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca26b4e2ab32dc0600d8b4c0a8446b2ce4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_SSI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca0ea40540a8ae621bd6b037313f141b01</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_SSI2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6c202eaad688d9b50a10dba5849167af</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_SSI3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca0deb692bbeba86e74a775a6421dc0575</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8811f698225b6f22675835384d7cdbc5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7611e5aa9f71a5ecd6f466dc090533f0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caba410e2e71ca9f59b76176672c9b1e4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caead627335ffc92260faac70ef409eff7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca84f22df0eedbb2ba90bc26bcb7d4da8b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac6e02b50494884bcceba6b24a9c26778</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_USB0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5363c94212e95049b036c487eff3e1d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_CAN0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab58916255b8946d66b0cf6a7c7ee18ae</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_CAN1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca61b240aefa01d614e06844f5b5943dc9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_ADC0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cae2455d48a21d53fe277cd3c5adf24d0c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_ADC1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca81b9534841bd60ff16455a28481d7a14</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_ACMP0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab043d91156620faf65bd2e7ba3b4892f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_PWM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5f8590f1bd8c06757c2ec264263d681a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_PWM1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca170fd02a576cd3e58beba221c63b995e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_QEI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caeacbf3016e47c9eb8042554666b3bcad</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_QEI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca0d25bd38b9d39e34504c068a85a66c45</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_EEPROM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8b25367eed70bf66750893969395d9c4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6383fd9f173243f9c552c63443a28f02</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca359d9e637ee81970457b6f2ad8ecf612</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca43c053c8a36d6ced3001bbc1181aa31e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6f8b1e2b9dce0bff8e294821af562aab</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caffb993127d5ad7ff814afd0b5e41bb67</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab7be18f91861ada479d141a71cdc4a73</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WD0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca046cc93f0c75e6a9f3c8d82296dbedc3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WD1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caedd62d0f4aeabc04c335f753ddd16675</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caab752e13fea766e909c1efb6d369af83</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6b6ae62b91785b7844c09efa72dba2d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caee4143c3631243bb89674442f966c8ac</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5dc46082bc7ebb37c3a0335a6e94aa13</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7caff6329167c3bdab800fb2a0d07122</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cabd8109243e642ce6d837c9f6f850f395</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca44f76e63809a5df30f158bb8cbfcb4af</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cad2438eef51dd7790d4faac2e461f61bc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab212b98035b8c1d7a52c66b8a8753ea7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1646b5369fa9b52ffe45deb801dffe53</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOE</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa4f1793bd6ab343073e151f470113a98</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOF</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac2f5731fc83d86f8f850c984c769ed97</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cadb087b1ddece696b0c76a7fc03e44de7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOH</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3869bb86c68604dc9015445cad5aa173</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOJ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6db1e5291f26eac23359ac9dae6d950c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cae33b8aad16ccb33ebba1d1eaa6f855f5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca87ce63fcd19271797660eee6b705d786</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca57d107799f9336f31e84b0cadd455e72</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPION</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca11a7252fd2351f7e48825fa6b209280b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caad3a48f5b917e478e276d137c0854433</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOQ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cade40950fd79bd4be332fd1a5a86620d9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_DMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa5659c38a81a48fd717ebde84b64aea8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_HIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cafa505d83b1672bacbab31c68d3b892d4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca96b4af86f966ab2d37b95ddcfd7ccae0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf0d58b6788b09b9b25ef36b180af6005</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca507b539a5e8127d359ff57e5c546a3e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1cafdc841e97ef1e45fac9f77cd67bfd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca27200fdd011075f80ec18eac21d811a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf2fdcaa5d35ac4f63fc12563ec64665d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART6</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa423f043367f931e13636b18fa6ce8fc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART7</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caeecfc516a3a6732cc6f05d38d66b9cfb</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_SSI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1b7f56125e323df352ae5391bb40e386</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_SSI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca2316ac607983f5bafef7f42aa821705b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_SSI2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa61fed56149618b50ddb926328a3e90e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_SSI3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca4fb5b1a1e53833917ba9862b11d85c44</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa8da3fc9fc64bc9bbbb2c0bd5c207ae0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caac1edecb6b1268334dd61d0be70eb8b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6e133432144b494871b9fc7434a91214</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cad1c9598c70b899240aa22be7b66e4765</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca866e650d47a300b010a0d6cf92986ef0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca22435962f8a22e2843a4fdcde07e8bb2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_USB0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3e2c89fb8b1a25854efa6173e3abbd60</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_CAN0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cafcc5638cfddfd30f1c4f0b72e7621784</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_CAN1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cace6a0c67861a7f9063364e3c23996b7c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_ADC0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8aeea8da59ca1b091f5ffe1fbe03350b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_ADC1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5000d6ca5e3b40a5865485dd8dac5ab8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_ACMP0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6d58998c9f779236ff93a78c89d20f92</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_PWM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab5b9111da3ae69f232b705e8bbeccc1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_PWM1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca53d2cc961160d1e6c05fa4e180656c29</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_QEI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca90dc05d2e2deb1bf75bd052269d9daac</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_QEI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caab3ed767192a9f1c572958d99e36422f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_EEPROM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3358317c6870dd5d3f344534992faf20</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caaa5749f1284d0fd60e24dceed3e19845</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca21e28d01f0c4aeb1c57918a47419419e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1b1772a391965c6c63491a4cc2199f33</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa6d69371c0aefa29ad28c84a8d2c5e1b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cae949847f800769a79798400b6276bc3b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf497645a2e76034377d94db5bffda572</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WD0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca66a2b58595f766c88a89456d92a6999b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WD1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf27b9bc9fbc7f398886d53051e1c7e9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf4e4e5d577b31124e1439065df8d0388</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca05457312f35ee02c99445845bd774cf5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cad3d86893051956ff70634657615333dd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca344f155fcccd30402c20051ea1aff24e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca451f2738d3868dcf745fe9a32eff995e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6bd98b42f2e9d897f9e2c30557fd3de9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab780a6472576a549c8282bd72424f6ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca463b8efb0b1d18397145221619849d10</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf93e5bd2886d5750ba8448f6def7f7e2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1e3b60573f8876b42812cd2533f43afe</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOE</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca052e00cabf7b5f4f879de58d1b800a34</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOF</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caeb39f15184175728c1f66ae9b2fc6368</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6a007341f0ec22ced18048a8d3dcee5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOH</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca32edc749039153efea9aec9cd73b5c57</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOJ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caeec160671808558a02f0fef5cf4421f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7cb603574124a5b4cad0dd4761082173</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cae06811c44ba8daf345c72fafe861996e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab52675509b66c5eb36a859554102f26c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPION</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca77191f653c4b3a8200759aebf37113a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca503607a6c996f27fe0dd7ca5b13a1090</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOQ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caefac3afd723c7499b7ee6bc024edcb1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_DMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cafdafb51e56845ffca6b8715f9f034be9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_HIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca10d534586668d3d60f4082a7324a7bd8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1e3076c6ebdab342bc02a25e1496c2ee</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caaab6521fa757e01bc08531474b332f2b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca41de1c88f35ceb3c07d35f671ad53508</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa73e22c8fcf45b72716dc6e19c3182a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab4aa5c7ee2f1d2001da02150b22d62c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5b30d06ea2c9c48b0550b7f48be0214a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART6</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca70e3a920c0cecaf2b5d17b8c9640c979</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART7</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cacd88c770eafbb36b4dfe11aec93eb29f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_SSI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca648f5b18454e1f34fe2b5f7a26787c1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_SSI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cafecbd79096060a7d5719ffd7fee65416</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_SSI2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac415fc33e675e3a0c7b9c83e283d8324</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_SSI3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3003b45b36dd2c510d9e7d2450701f39</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caee1726dc7502226128f256d5888c3fe7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca230e1ad541a3279fbda81dbf6d160a3c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca790e86296a06d23c43f629410741ec17</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab8c14dd3cc4990cb7c078208458a850d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa5980152513b981d5257c21194e256e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca56f9aef0fa4a67fe5e73db5e251818c1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_USB0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8f353ad79c2bae6065a14d0912dde03c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_CAN0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cafa65ea4c7879f7b4362f380cab768981</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_CAN1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cad12005b67225647d17685c1f4d5172d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_ADC0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cad5ab3be3260e7f2063c5565f63509d28</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_ADC1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7aff55fefdd807626eed54b113497898</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_ACMP0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac174433556444d8b81c462697d008a43</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_PWM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7579e619e6f15718ce4632ae0c5002ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_PWM1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca9bfbba89e407bf5acb591a071df5bd43</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_QEI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cabb79eacfc68137024a6dc3f7756e8f0a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_QEI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac0b3e282db4a0066e7a770c6f0449d56</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_EEPROM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab1db2619cdcfb45c70fd49196b179961</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca215fbe3cc93bb7378d5c1c3fc30f8463</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca86630e306040d178a3f981156ad3db0c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca18eb7b7308cfbfe58ad6905cfabceeaa</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8923e4b18bf6c1df5d744a022f87dbf2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca4049b8d01dbe4fe6ad17d4af74ad4757</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca43d42bf96f91a44c4fb69aa9b21e7bad</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>periph_clock_enable</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga82ac341aee840ff3b2fc15d79de016c9</anchor>
      <arglist>(enum lm4f_clken periph)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>periph_clock_disable</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa8ca86965f943013fd3bc6eb61f889e2</anchor>
      <arglist>(enum lm4f_clken periph)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>uart.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/lm4f/</path>
    <filename>uart_8c</filename>
    <includes id="uart_8h" name="uart.h" local="no" imported="no">libopencm3/lm4f/uart.h</includes>
    <includes id="systemcontrol_8h" name="systemcontrol.h" local="no" imported="no">libopencm3/lm4f/systemcontrol.h</includes>
    <includes id="rcc_8h" name="rcc.h" local="no" imported="no">libopencm3/lm4f/rcc.h</includes>
    <member kind="function">
      <type>void</type>
      <name>uart_enable</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>gacd6ffe5e9e3f555d07a3af7ee7fdf74a</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>ga0167757699a14269e55640b5034ffcbf</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_baudrate</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>ga05a4513fed24167dd33ebc5805b9ca77</anchor>
      <arglist>(uint32_t uart, uint32_t baud)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_databits</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>gadafb5a1c4596fb687bdc99dff191d88f</anchor>
      <arglist>(uint32_t uart, uint8_t databits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_stopbits</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>gaf3ddb4b6b173fb6cd30139ac67ca62cb</anchor>
      <arglist>(uint32_t uart, uint8_t stopbits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_parity</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>ga7b0acac78989b94dab26d92d1b640e05</anchor>
      <arglist>(uint32_t uart, enum uart_parity parity)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_flow_control</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>gabc37af12e01b5827d40f5ed7d149076f</anchor>
      <arglist>(uint32_t uart, enum uart_flowctl flow)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clock_from_piosc</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>gab9d2f5223bc6f08621e9b6d38f8a4fb9</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clock_from_sysclk</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>ga917d75f562fe93eea7c684756f9f02c1</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_send</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga225444640f9510f126a597004fe3dcfc</anchor>
      <arglist>(uint32_t uart, uint16_t data)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>uart_recv</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga58e1b434c54c8994d9120194e2b542ab</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_wait_send_ready</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga9546f62d7bb1695a34748ae418d2ed7f</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_wait_recv_ready</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga42ff0fbee0d1b80cc77667c14c3213dc</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_send_blocking</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga2a621068c90e83ee0d9646235dda7ba6</anchor>
      <arglist>(uint32_t uart, uint16_t data)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>uart_recv_blocking</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga5a7498f353526e8cb9e6c8bc5735d1f1</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_interrupts</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga20cf32aae3969a9b1e857cd1ca145a27</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_interrupts</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga2fdf35d91a90e2452368b0358d942f93</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_rx_interrupt</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>gac40fe4cd2b2d331d83100e305c2f6f78</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_rx_interrupt</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga6c46b4228c1da4010f8e8d5a66bde0c0</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_tx_interrupt</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>gaab2f52c4fdb9b8ba9e89670e93727ecf</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_tx_interrupt</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga2d6559da0d940412e88374af5fe8a5dc</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clear_interrupt_flag</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>gab5fd7ee81bc609e4aca33a4b9079611b</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_rx_dma</name>
      <anchorfile>group__uart__dma.html</anchorfile>
      <anchor>ga1028a66c1d0794aa79c9f9c3f116f124</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_rx_dma</name>
      <anchorfile>group__uart__dma.html</anchorfile>
      <anchor>ga0260bfb6563f9ffde298d76e71265994</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_tx_dma</name>
      <anchorfile>group__uart__dma.html</anchorfile>
      <anchor>ga8efe31511341bd42f7ed5c60aa4a29a2</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_tx_dma</name>
      <anchorfile>group__uart__dma.html</anchorfile>
      <anchor>ga92c430ac58b1716402b67ec2ab3be426</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_fifo</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga01026f3f8ddd0939fd8e291ea3837365</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_fifo</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>gad0ef308eb0c6b5a6009c97b3127d014c</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_fifo_trigger_levels</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga37814e8f7102ff91e9e73d17ffb0e55a</anchor>
      <arglist>(uint32_t uart, enum uart_fifo_rx_trigger_level rx_level, enum uart_fifo_tx_trigger_level tx_level)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>uart.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/lm4f/</path>
    <filename>uart_8h</filename>
    <includes id="memorymap_8h" name="memorymap.h" local="no" imported="no">libopencm3/lm4f/memorymap.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>UART0</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga0508661f121639ffdee7de2353a0def2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART1</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga8d69bf04d07af4fbbab5a8bd291f65ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART2</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga7f6bd6eb89ae2eeae97af4207ebe3cde</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART3</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga961726a611b38bcaf61f3d598b0a59ec</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART4</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga7c035f6f443c999fc043b2b7fb598800</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART5</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga9274e37cf5e8a174fc5dd627b98ec0fe</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART6</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga61531d783c55be06875e7029331ffd67</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART7</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga20bc10f5b73e8b51724b2f23c5b2e785</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0c98d6b342b60683f8ab1be7fc4cfe67</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RSR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga22b6ff089fceba4538995f696d674f93</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_ECR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4455df69f03245da7e55e1e6f66e9f51</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gafa91f42e4c05178702d91ab5a044ed3f</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_ILPR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae2882a6a4b8b962ed7d13c57ef61418c</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IBRD</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga56c90494604bf4236327dfc604a70bec</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FBRD</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gac2699a574273c40349ce203d84ea098c</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gafb713a577cf19294540b99d005239012</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga481e29e3dd902b257304a54765c631d7</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaa97170d41c922b22fbcee85d846d9723</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaf303f95e542d800b839da69a164b48fd</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RIS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gafa4d3c48ac3fb9156570f0d3869be6d6</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_MIS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaedd0d984219cde25cc9bd4c98df53edb</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_ICR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga386e0b9ad35c2153352035406c95bfce</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DMACTL</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2df54bef23a4d6b5f04911898c05d615</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaa015d4be34d27b7efedfa70c2bc47dd9</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LSS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga5033a846ffa37746dd420f0abbd192f5</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LTIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gace0ee38f677ca6fb5e087ee5f9471387</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_9BITADDR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab9be99b9a61ea4ec1971ec04aaa92647</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_9BITAMASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga59bcdcef048c7038e01d209eff6871ab</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PP</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4d571827104805cbcff1546b4597e060</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CC</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga65cb0afad62b63adee375cb2100d8078</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3a626dd544a0c862c8d4a05cb6fa0568</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID5</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga07274a1430cf6020433edb13d4efb69d</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID6</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga94a1b773bf4e75927c4c4c5379057d00</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID7</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga30fcf1da9dab8ef0b33e9937cecd020e</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID0</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab3214df5e67cc4704e9eee9ebd95f3fc</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID1</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga828f5a604381369427c2f38f0c928151</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad27b8abe99a87eee9991ff7f9eca6d3d</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID3</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae012a9706c5138fd2fb48622ff1d1379</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PCELL_ID0</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga9978829c0bd2ff2ecf4804e2e34b9b07</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PCELL_ID1</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga444cae1f835a85f653b23116fda02630</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PCELL_ID2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga222ae22f0a5052fc95be975d08763140</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PCELL_ID3</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8b9b032313b6d2299152397c9cd6be91</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR_OE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaddc33d9ca903b5498498845fedcc2406</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR_BE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8abd67385293e64f5736e5faddc68909</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR_PE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3ce57f5f6c7670322e73a4156223a03c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR_FE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga759524d95ccfeb42ae6973ef1e727e92</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR_DATA_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6e9ca2dfc3ca7f53c6c217784c0f298a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RSR_OE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga7d66e764b50faba0d5327e912b6d85a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RSR_BE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga59aa21bf1e8bbec3739106f17e956188</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RSR_PE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae017de851d1d1433b4b968b74ed4446b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RSR_FE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad9663ec3a20819ba90fc7c8e6a3c17cf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_TXFE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga08ea2055746abf83b7336ae08dd1c92d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_RXFF</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga842ff6a0449123ede0b5b93425ce902c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_TXFF</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2f6a08ae8a3005e737005cbd607081b1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_RXFE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga9ba067e6425a6c5b5aca79874c549364</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_BUSY</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga39a3e9403d1914dba75ca838fdc73364</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_CTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3f708d80e15117726f4faf915fc8c349</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_SPS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga44c6291aea20dfcb8d75fc9d47c1ee10</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_WLEN_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2402827931937dadf399741d30b83e09</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_WLEN_5</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2b26de9efce73ee468f7060ef685bf85</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_WLEN_6</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6458da85c7d1c15e3f1b6a70893ab906</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_WLEN_7</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3c312edc8c30df376cbb7a702d799c12</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_WLEN_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae0e27af519dfbffe6d6a50942bdf30f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_FEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gac63fc7bdadb98a24125de76e1468510e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_STP2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3aa637fc03bb39a175932d19c48e2e5b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_EPS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaff1a8a6c54483dbf84d1902397d47b7b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_PEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaf1d5a9b375d10c260bf2e7b85bcbfe0b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_BRK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga80da31de1b31f9d4cd25906e43a9a919</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_CTSEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga25c8b58bfb4a36de3f897c2863888856</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_RTSEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga75a515e952135c324c71b86c1c39991c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_RTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gabfa4cdce464b8d60caafc85a577cc2d4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_DTR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga64e1cffe33d4ea911bf3f32bf6baf584</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_RXE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4d522022557e403572e518db25b3cf5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_TXE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gac81859db681e3918f88e0f7aea596a06</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_LBE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad6ac64f4d5b8d6377bfd1d3799813710</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_LIN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gafe830fab8bf36f567e24c9b09f8660df</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_HSE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0e25d885a9f3a9afed6cae6cbda20273</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_EOT</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga94ba876fdc237d15d664750dc8035264</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_SMART</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad1df1c7e40e3fb26b09e99f3a15105f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_SIRLIP</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaec5956fa3d9f55bb1345fdb0cb7b7800</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_SIREN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga5b9bcd2c0a0180f3a61c1aa0afa36697</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_UARTEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3d170d515d0d3082a4a4720c2a8c4fde</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0f71f525c36b1043fa2c3c724de52799</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_1_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad35fd9fe2144098c33ed0e6fea59bf59</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_1_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8ad9b3a77cdd5bfc377e56417351334a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_1_2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8bdfe54997f9d323be44777226a43379</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_3_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga625f6e1af03073f7a09874c9586e34f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_7_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaca1bedd6d2af97b2f54337365cfcc575</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3f91489b140ae6f2548b356066403b03</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_7_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaced1ed4db0e884f3baa7c4bc6bec37e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_3_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6e6afe4ba438f753fe8af611ceb5b225</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_1_2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4fe2390470f14c8fcb46696182667908</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_1_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8bf3308a27ce97a489c78f1568362082</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_1_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga81f70b7961b17d393bfe0c96d725291c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_LME5IM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3b11e7cfa01c185335cb158f34db2ba3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_LME1IM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gabde8ea82682a67057ad6541d737c9451</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_LMSBIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaedf14b595013f396376ba168660967f2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_9BITIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae7a05ad05d5e3ba626aa112f6572a83c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_OEIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga31eb54dcaa6a8b8f0cfa9b2546ecf90a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_BEIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga46f192668edb0a97e140d59d561f8018</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_PEIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga82fb24baaca8bb73091c345efce113ba</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_FEIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaa1bc63532f94fa71e77b820c211642e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_RTIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga527c259858bfb707366288c0469c38e2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_TXIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2a390d47a4075123384c44717159d4bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_RXIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gabad98dd5a214f571cb0dca49383fed17</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_DSRIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4c491dc6be46ec71bbffff12bdcedc7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_DCDIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3c41556d8182126dbaa32460313e5448</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_CTSIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaff361cbb8eb14b4b8e7af5038ec6675b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_RIIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gacb7bd7884ad0aa579942724b83678bbf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DMACTL_DMAERR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab1fd3871951b17e306c5e616f0c131a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DMACTL_TXDMAE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga237d4a08c2819e668fb7d7fb5ad308cd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DMACTL_RXDMAE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad01fa53a8abced4fbc821b3c34289d1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_BLEN_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0891c8cc859b3f8077cd08a6c47b846a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_BLEN_16T</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab30fd25f25590abb6c2cb1e1fc56cca9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_BLEN_15T</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad8da29741fc4d2fbf008b38ea94287a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_BLEN_14T</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4bf662defab18af97f5973c232ec08f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_BLEN_13T</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga79a1ceca660d027fe4743a290816c0f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_MASTER</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae23b5e9084af31d3c231a4adf4b5a0ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_UART_9BITADDR_9BITEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6626602aac55324657ffc813e648929f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_UART_9BITADDR_ADDR_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaec1129baa25b7a59e3b310414f00cdb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_UART_PP_NB</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga5d450e8bec95b40e6cb5b9fa3c5c251b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_UART_PP_SC</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6f426020f7f410b2cc85a8fd17814189</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CC_CS_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaaeadf2fb733578296ea3699644173773</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CC_CS_SYSCLK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gafc8eff880490901a538ee6ea23364d18</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CC_CS_PIOSC</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaa7e2d80dd63bb6eb2dd73ffae92fa6bc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>uart_parity</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaef0010d952178bd0d851bcf3b63daeca</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_PARITY_NONE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaef0010d952178bd0d851bcf3b63daecaaa80d2d8ea61454045ebe71d155e85b3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_PARITY_ODD</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaef0010d952178bd0d851bcf3b63daecaad90cc425f5ba447773a44a75be6593e2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_PARITY_EVEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaef0010d952178bd0d851bcf3b63daecaad908a637b1dd23f93b149dd2d8bdfdb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_PARITY_STICK_0</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaef0010d952178bd0d851bcf3b63daecaa38e4b2e7c5b38024a01f2195748b77f3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_PARITY_STICK_1</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaef0010d952178bd0d851bcf3b63daecaaf20f92778e29fd61d30d276c03353085</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>uart_flowctl</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga50eb7d8c1338b309a11dda5e86240706</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FLOWCTL_NONE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gga50eb7d8c1338b309a11dda5e86240706a087e42c5229a10b6b02aa7f3548e0e18</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FLOWCTL_RTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gga50eb7d8c1338b309a11dda5e86240706a8105549da2b48f557f98e5575dd48fc5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FLOWCTL_CTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gga50eb7d8c1338b309a11dda5e86240706a0d9da2df46dc2f980c019b5c0c955a2c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FLOWCTL_RTS_CTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gga50eb7d8c1338b309a11dda5e86240706a8e06515da93c430fa99ac8838772040e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>uart_interrupt_flag</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad02c1455d23262a6dbda84d6053d1f7e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_LME5</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea22d9d80af21c49fd2845e65ae8964dfc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_LME1</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eabea77aada0f38984ab084b876d26e23c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_LMSB</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea1c2452c3b449ba88c7796845815a974d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_9BIT</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea8775b5683b20ec15d876196c580c584a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_OE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea9de058b4403afc1fa2aa7099c45d0b06</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_BE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea163453765b5cb996248cd1145c1dac7b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_PE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eafcfec516eb93a5484263c358cfe27fe9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_FE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eaf222be4a67ad2277a89bca5696d17fbb</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_RT</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eab0ad539c76928c0113725547575376ca</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_TX</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea2a145ce488762550b48d111c76e03a60</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_RX</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea32eca039b291092af8df630b6ea6613d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_DSR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eab03ddef3181bfb7cb0e11bfbb8a42950</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_DCD</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea26117cec3a3d09416668f45aad1ae08f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_CTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eac8355e07c854f7e22921e01bbc17d5df</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_RI</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea19236a41b0921ff78c01940fb02a9654</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>uart_fifo_rx_trigger_level</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gabb9ce81df1867ef09b554e02c63f9be5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_RX_TRIG_1_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggabb9ce81df1867ef09b554e02c63f9be5a9a9d22fe9ad81f42dc7dd48e3c0522f9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_RX_TRIG_1_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggabb9ce81df1867ef09b554e02c63f9be5a698200c109ea3fb1035cc81ff5c3f34c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_RX_TRIG_1_2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggabb9ce81df1867ef09b554e02c63f9be5a370cf31f3500cdbd65b9d6acd71efc2e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_RX_TRIG_3_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggabb9ce81df1867ef09b554e02c63f9be5a4f86835a14f9d693d8982534bc2e50e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_RX_TRIG_7_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggabb9ce81df1867ef09b554e02c63f9be5a73fd9cb5457a327d74fdf727be6f580e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>uart_fifo_tx_trigger_level</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaf79e87f1a6f23cc0501f4d2e3667a848</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_TX_TRIG_7_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaf79e87f1a6f23cc0501f4d2e3667a848ad73b67cad4b485679cd70f2af3fa73f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_TX_TRIG_3_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaf79e87f1a6f23cc0501f4d2e3667a848a0ba87eebf5636a89499d1c75f4524fc5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_TX_TRIG_1_2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaf79e87f1a6f23cc0501f4d2e3667a848a6cf07bc1412bbf702f39ad478add22fc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_TX_TRIG_1_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaf79e87f1a6f23cc0501f4d2e3667a848a3b6765a42c9e8794949062283819221a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_TX_TRIG_1_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaf79e87f1a6f23cc0501f4d2e3667a848a38c34c00f7ccd0a9424cf3d48338a4c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_baudrate</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga05a4513fed24167dd33ebc5805b9ca77</anchor>
      <arglist>(uint32_t uart, uint32_t baud)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_databits</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gadafb5a1c4596fb687bdc99dff191d88f</anchor>
      <arglist>(uint32_t uart, uint8_t databits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_stopbits</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaf3ddb4b6b173fb6cd30139ac67ca62cb</anchor>
      <arglist>(uint32_t uart, uint8_t stopbits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_parity</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga7b0acac78989b94dab26d92d1b640e05</anchor>
      <arglist>(uint32_t uart, enum uart_parity parity)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_mode</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga35e0dc6cb6910a92c425b6303006e273</anchor>
      <arglist>(uint32_t uart, uint32_t mode)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_flow_control</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gabc37af12e01b5827d40f5ed7d149076f</anchor>
      <arglist>(uint32_t uart, enum uart_flowctl flow)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gacd6ffe5e9e3f555d07a3af7ee7fdf74a</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0167757699a14269e55640b5034ffcbf</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clock_from_piosc</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab9d2f5223bc6f08621e9b6d38f8a4fb9</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clock_from_sysclk</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga917d75f562fe93eea7c684756f9f02c1</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_send</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga225444640f9510f126a597004fe3dcfc</anchor>
      <arglist>(uint32_t uart, uint16_t data)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>uart_recv</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga58e1b434c54c8994d9120194e2b542ab</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_wait_send_ready</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga9546f62d7bb1695a34748ae418d2ed7f</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_wait_recv_ready</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga42ff0fbee0d1b80cc77667c14c3213dc</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_send_blocking</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2a621068c90e83ee0d9646235dda7ba6</anchor>
      <arglist>(uint32_t uart, uint16_t data)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>uart_recv_blocking</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga5a7498f353526e8cb9e6c8bc5735d1f1</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_rx_dma</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga1028a66c1d0794aa79c9f9c3f116f124</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_rx_dma</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0260bfb6563f9ffde298d76e71265994</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_tx_dma</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8efe31511341bd42f7ed5c60aa4a29a2</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_tx_dma</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga92c430ac58b1716402b67ec2ab3be426</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_fifo</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga01026f3f8ddd0939fd8e291ea3837365</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_fifo</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad0ef308eb0c6b5a6009c97b3127d014c</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_fifo_trigger_levels</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga37814e8f7102ff91e9e73d17ffb0e55a</anchor>
      <arglist>(uint32_t uart, enum uart_fifo_rx_trigger_level rx_level, enum uart_fifo_tx_trigger_level tx_level)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_interrupts</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga20cf32aae3969a9b1e857cd1ca145a27</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_interrupts</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2fdf35d91a90e2452368b0358d942f93</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_rx_interrupt</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gac40fe4cd2b2d331d83100e305c2f6f78</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_rx_interrupt</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6c46b4228c1da4010f8e8d5a66bde0c0</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_tx_interrupt</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaab2f52c4fdb9b8ba9e89670e93727ecf</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_tx_interrupt</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2d6559da0d940412e88374af5fe8a5dc</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clear_interrupt_flag</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab5fd7ee81bc609e4aca33a4b9079611b</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_tx_fifo_full</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga509d7ccd81a766a5a240558f85e3ca0b</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_tx_fifo_empty</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>gad1246f6b0b156022d942ded6a7a7e398</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_rx_fifo_full</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga6a2584c859c685b3ff9693c7e9b1d657</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_rx_fifo_empty</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga840603de8127f2b63abdd1f5d113f938</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_interrupt_source</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga6271114e5fe89f29dd5243c7c0c1934d</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag source)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_tx_fifo_full</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga509d7ccd81a766a5a240558f85e3ca0b</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_tx_fifo_empty</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>gad1246f6b0b156022d942ded6a7a7e398</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_rx_fifo_full</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga6a2584c859c685b3ff9693c7e9b1d657</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_rx_fifo_empty</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga840603de8127f2b63abdd1f5d113f938</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_interrupt_source</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga6271114e5fe89f29dd5243c7c0c1934d</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag source)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/lm4f/</path>
    <filename>usb_8h</filename>
    <includes id="memorymap_8h" name="memorymap.h" local="no" imported="no">libopencm3/lm4f/memorymap.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>USB_FADDR</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad92d008d03f5d8f09f16181e9e52e876</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaef1a9dc21299c653d5abf0dc1a27a502</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXIS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga03e4c678a0ddae17ca23e85e65f26814</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXIS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf2c9fffa60198a23c882e7ddf8549d83</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXIE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga743c579f79e4fa19802f53437e1e1ca7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXIE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga5ff4ac77ee784c979df2860a64b8d106</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gafffdb3b9440c9a4e26ff84ff533f573b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab4e58ccbade58db72ac11981737ec51d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FRAME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa4f674d9fa6558a47c8f89b2a45b49b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EPIDX</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga5950b670d8ea2f0a93627f84fdcb402a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TEST</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga64956051c35bf0ef75d23c974697e078</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFO8</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga38333c938233a8c2ad9d3c19f3bb1e2d</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFO16</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga6f48d071c29984bd3bdb53f92189c14f</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFO32</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga6492869d57f9cfd220a782ff7833ba0a</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXFIFOSZ</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf5861a491d3a9671ed07dbc2db38e781</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXFIFOSZ</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaed8243b4826430be9c4e3bee9f8429c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXFIFOADD</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga96d11af3bcb3340682b9dfee288c8c09</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXFIFOADD</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad961db7bb51ee9ff79b5edb6d641c622</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CONTIM</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga0fbe0d83e408acf19c0f304081e71483</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FSEOF</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab15e63a3693b6703c87b9b2adffdd2b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_LSEOF</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gac116c270b960020a0a37f42b5cc32115</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga6ef57c5869b4b001a49f327c1d33f166</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRH0</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga576c2ea5c5dd89ae4752036e63722a05</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_COUNT0</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf7ecfad56454fbf1f8192323bdced3aa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXMAXP</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf0769e1a61a5341db94173a0acd332c8</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf324f8096f3711bdb9580b294157dc60</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga27de1c7931c1f9ea6021fe6581ac66c6</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXMAXP</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga7b0b108a19aaca9923d40c5801dcbac2</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga50c1e6369d5fdb2971a17abaa5d1e8b5</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga1b512c91287f77cb42345427b8143ace</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCOUNT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga3ad7f9f4d763a4736ce92aa59eb590b8</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXDPKTBUFDIS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gac3f01ef4204c55f6c827476bf53dc354</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXDPKTBUFDIS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga1ebfcf49d3be483454d870c6044c335d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRRIS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga43be47e6b25da4e00a0f9a42f33e69eb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRIM</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga85292bcffaea0579b45c5c8202da7ac3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRISC</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga12db7a2f100c32770a3507dc7b35cc88</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DMASEL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gafce8c301e416593b42c5ffbf0b430563</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab123b4a1a9f145fcd93c9612044cf585</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FADDR_FUNCADDR_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga2d13500deda4ef0d8907b4a1eb004da1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_ISOUP</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf571b356b470125bebacd49b2a9ca638</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_SOFTCONN</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga924294c1d15dc9e28ed111eea2e24afc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_RESET</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga07be6b6ae6bc5b8829d50f4364e72170</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga1b7d36d2650b70ee144894419244e4c8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_SUSPEND</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga26f24c171021e851e56f6bc09ce30e1f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_PWRDNPHY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga173a03da0b3497bbce7301e0a41c06f5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP7</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga58c090fa4a5e2d9963d163f7fe18e732</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP6</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaaed43c78cabd4bf8b55914cbf7942335</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP5</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga3614760777b40aa0f2125acf4fbefc5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP4</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga60874fded11c6c5d491a5e47ce185ea3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP3</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad2dd7082012d04a39d15368605a7b072</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP2</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga487de857d562aec62e81e9bd0ab11ad4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP1</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga8ff7e6f3fb1426d9d0bb2eeb4ff20e4e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP0</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga7148f9b5726dedfd6a36f52fdb9ce66a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IM_DISCON</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga7d16c57db8f5544d983924d2234dc79a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IM_SOF</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga9b6c0d1dc4e39220ec26dcde2f174441</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IM_RESET</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga988539e74865cb5e79c49ab0acc30684</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IM_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4d82a47e24cbcfe0a89c9aa961b0048d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IM_SUSPEND</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gabca1735f53faf4a44593ef12e1c2bfc3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FRAME_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga9631e2d739ae0d4b9dfab5dbe331a49d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EPIDX_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gac9a64b85a8fd5eb11a5f7f4585883f66</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TEST_FIFOACC</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga8ef6bcfd81ebb6bd35155adcc311755a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TEST_FORCEFS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga55a48b6aae7b23e5dd88dc7f169b33ef</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_DPB</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf21c7990149621025fcec87431139718</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaddc4a3f6756a16013069a4e95b097de0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_8</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4c564ad6cb10c62d559eee1c1a9c9bd6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_16</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga994491866569641e236d75a626d9c591</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_32</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga835a73b30026bb2f107303a9204b4b20</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_64</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga21636367bc81b16e1fba5a827ebb446b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_128</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab8708d0c9dc528de2f5e9f797d1f4b5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_256</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4b7bb7ca245ba2a1dcb413b0bda30060</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_512</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa3c29579f391a7692b6cbbd6a0772a51</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_1024</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad8e6120f36226fe9c6bfa1f8efa7ecf6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_2048</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf7774421e9038191ea20b0ef6fd2d5c7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CONTIM_WTCON_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab505befaaba2947c414d796373aecbc0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CONTIM_WTID_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga79c97c91a179d566af79a876c15f4424</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_SETENDC</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf99ad022830386b6eeec1668cf509228</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_RXRDYC</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga48e2f3edca2d034139b6ee968e041d34</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_STALL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab41d8813224b450b8d5fb8602d7f8362</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_SETEND</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab649bf662d9310d93032aadde771d022</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_DATAEND</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga9d0b019991becfdfb91fe4227daad63e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_STALLED</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa0f1b7e9ac27a773a386af520e89980d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_TXRDY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga66cb7a914c02dce1716cb0b6669da104</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_RXRDY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa46f20c660cc47f9b3d8f9feace51914</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRH0_FLUSH</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gae6749e9d7f8d7b2afaa90cd3fa1e50ad</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_CLRDT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga9193bb8711914d2e9dde021851048bdd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_STALLED</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaba32c0c3e09bae0fbe4c95175f739775</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_STALL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad77a762d47d7e430638ef7766ba9c73c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_FLUSH</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga6da8bfded6fba86ebdc00cc81392988f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_UNDRN</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad8d302684eac7340cd649f65e6ffb72a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_FIFONE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4f699992cf30339abe4b680d77c98698</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_TXRDY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4bba6046fa89a27bc00833c00f4ead3e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_AUTOSET</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga663fce4e5169fad7ef3db84585a96faf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_ISO</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaac2f13d9dea99747a60ff3627d0e6e04</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_MODE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4b6f1deb46bd6ea8273fac808e11d008</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_DMAEN</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga10b8ec666f75c0efd120c63639046a01</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_FDT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga5a9c4bd6b01cd885d6a50d4ee7a3d2a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_DMAMOD</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaad543c87d547913611c0c11327c1d7a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_CLRDT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga096ac7b32487adc81e7148b608b5aa7c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_STALLED</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga66c90350c2da3cac2f95a1af1bc00efe</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_STALL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga1ad21c3fd83c33cc42724833ef965597</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_FLUSH</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga21bbfb545360dca03a05fa70a6e060ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_DATAERR</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga2bb6163609f40338cf48f6cfa1e7f0f1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_OVER</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa5bc944309abd867cee26f41d4fd014b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_FULL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf2cbaaa37b7bd4220dd808434178d3bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_RXRDY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga3642595bc567347f2de41412c8f788be</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH_AUTOCL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga031c4b2c98827bbce50f5f25d3863ce6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH_ISO</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga850bc5c228e3200419bf97ac080161f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH_DMAEN</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga772c7bfe52b1a13810d9a685f50ed0ec</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH_PIDERR</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gae2513db8a7ef5237a5b38245a5e909b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH_DMAMOD</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa259ba84966b3ad1925e7ef571effbb3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRRIS_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga64968f0eaacb51f2ba574ae7691dff83</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRIM_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga94d7abb9de415ca2387b3dffb10a4fe0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRISC_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gacb49a37729f80d7ca8d7f9b241096522</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_ECNT_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga09a4247eecdb0bd2ffd01d9d7d89551f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_USB_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga9e33ae0ff5e70604f711bc6f391524ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_USB_NA</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf2ebff7652c2fe18b88f28dde7337c68</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_USB_DEVICE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga227db2fe354d138410c90000895bdb97</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_USB_HOST</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga1bc6803dbdc5917c5f8958a00b4cbe91</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_USB_OTG</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga28caf357c58dc1fd14a751eb37591754</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_PHY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga54656f02a50d670f35ea6fc707ce956d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_TYPE_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga7a5b770f09b525fb6cd581f5c87035b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>usb_interrupt</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga0f2f22f2ab06463da36ae9c6c2a63ea6</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_INT_DISCON</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gga0f2f22f2ab06463da36ae9c6c2a63ea6ab7203dd6b18353416e992310394ed995</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_INT_SOF</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gga0f2f22f2ab06463da36ae9c6c2a63ea6a0d3be5f3ef316460b344e80ead93a4e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_INT_RESET</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gga0f2f22f2ab06463da36ae9c6c2a63ea6aa45692bc816bdd9bf6819db51f98ad3a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_INT_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gga0f2f22f2ab06463da36ae9c6c2a63ea6a3d7fca27fa1e3edbc72a0b278398ca42</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_INT_SUSPEND</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gga0f2f22f2ab06463da36ae9c6c2a63ea6aa36cce94b72cb32846cb6478806e933b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>usb_ep_interrupt</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf0a19719081410c9b09041a7dd65cd11</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP0_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11a93a6bce8bca9bc79fb14552c840ae41d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP1_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11a90bb9c7bfad2314c174df49c83129a3c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP2_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11a4325207f3d1ac8680292e80fe4361c51</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP3_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11ae7eeedf54e147181ba987a244951154e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP4_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11a188f2d830cb08f1eaa8ca64d6fa90510</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP5_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11ae49ed0b247c7d8c63f13818096e5218f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP6_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11ab578ebf1ba7080c3e865681a91da0192</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP7_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11a9735ada9074d7afcc7fbaee751b8e6a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usb_enable_interrupts</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad6acfc8a14795d10ae71053b0cfbf717</anchor>
      <arglist>(enum usb_interrupt ints, enum usb_ep_interrupt rx_ints, enum usb_ep_interrupt tx_ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usb_disable_interrupts</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga86efee4380227664681caba4d4c2a010</anchor>
      <arglist>(enum usb_interrupt ints, enum usb_ep_interrupt rx_ints, enum usb_ep_interrupt tx_ints)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb_lm4f.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/lm4f/</path>
    <filename>usb__lm4f_8c</filename>
    <includes id="usb_8h" name="usb.h" local="no" imported="no">libopencm3/lm4f/usb.h</includes>
    <includes id="rcc_8h" name="rcc.h" local="no" imported="no">libopencm3/lm4f/rcc.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>MAX_FIFO_RAM</name>
      <anchorfile>group__usb__file.html</anchorfile>
      <anchor>gad5f9b20acf4cf9b527ae4463b4940971</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usb_enable_interrupts</name>
      <anchorfile>group__usb__file.html</anchorfile>
      <anchor>gad6acfc8a14795d10ae71053b0cfbf717</anchor>
      <arglist>(enum usb_interrupt ints, enum usb_ep_interrupt rx_ints, enum usb_ep_interrupt tx_ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usb_disable_interrupts</name>
      <anchorfile>group__usb__file.html</anchorfile>
      <anchor>ga86efee4380227664681caba4d4c2a010</anchor>
      <arglist>(enum usb_interrupt ints, enum usb_ep_interrupt rx_ints, enum usb_ep_interrupt tx_ints)</arglist>
    </member>
    <member kind="variable">
      <type>const struct _usbd_driver</type>
      <name>lm4f_usb_driver</name>
      <anchorfile>group__usb__file.html</anchorfile>
      <anchor>gaedb3ad0d80317ac7e69d6e616077df06</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>LM4Fxx</name>
    <title>LM4F</title>
    <filename>group__LM4Fxx.html</filename>
    <subgroup>gpio_file</subgroup>
    <subgroup>rcc_file</subgroup>
    <subgroup>uart_file</subgroup>
    <subgroup>usb_file</subgroup>
  </compound>
  <compound kind="group">
    <name>LM4Fxx_defines</name>
    <title>LM4F Defines</title>
    <filename>group__LM4Fxx__defines.html</filename>
    <subgroup>gpio_defines</subgroup>
    <subgroup>nvic_defines</subgroup>
    <subgroup>rcc_defines</subgroup>
    <subgroup>ssi_defines</subgroup>
    <subgroup>systemcontrol_defines</subgroup>
    <subgroup>uart_defines</subgroup>
    <subgroup>usb_defines</subgroup>
  </compound>
  <compound kind="group">
    <name>gpio_defines</name>
    <title>General Purpose I/O Defines</title>
    <filename>group__gpio__defines.html</filename>
    <subgroup>gpio_reg_base</subgroup>
    <subgroup>gpio_pin_id</subgroup>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DATA</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gadf02597783c151627da209a001fd28db</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DIR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gac3ff1d81b6ee02147458f7741b9877be</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_IS</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaf3168523aa71db397cc6b90d06c19a5e</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_IBE</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga93047ff801b7bf6f8296706703f2bdeb</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_IEV</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaea367cbbe8e1040667eef8b89533d954</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_IM</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gae19924963589e8d9ae1ce5204c92afe1</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_RIS</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaac5a97bf3ce4743eb2a15016d02f05a2</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_MIS</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga0a9247b9076a5e068506e4778bd7117c</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_ICR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga7f7bb0f9d7f416c02e635d7fae773c66</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_AFSEL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga4af3c414275112a06dd5e02ddbe94252</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DR2R</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga0386cc42ff6da71217fb156a7410fa75</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DR4R</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gad1ea20c9c60d5bc22817c71de40a7d61</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DR8R</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga3a8a81a8d969e2dddf535005b8d0d0d1</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_ODR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga58655e9db856a8f3208e388814db2de1</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PUR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga85534bcd71ffd7330699c2a19255bd8e</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PDR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga91a11047ca48e9bc9d8b4b0b72feb00e</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_SLR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gad7df547f4a985cd1ecf74f052767fd0c</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DEN</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga7eb35658e871ace9d38772762765f964</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_LOCK</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gadbed78649b4858807f28bd9f4b30cbf4</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_CR</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gab033a90c9505838734359175c38ef15e</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_AMSEL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga8a312414a741e68773f02828db9e6d85</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PCTL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gae0a7b9acf9e36985689893dc8cb6e9d4</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_ADCCTL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gabfd4b002e858cb8430cd1825b71979fc</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_DMACTL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga3fefaf90062772262aa8b3f2caf32406</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID4</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga043d38c836fb8896eb7e2ffa125bfead</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID5</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaf4090988152ba51816c7dd3449e741f9</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID6</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gae829c3b0903409f010f4fa995dd159a7</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID7</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga96c6d6cf6ccbb9b0d23441360c158d53</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID0</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga43ce567a69a392019261456ff8ab3dd3</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID1</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaf7dd2186df64e01fe54402b3cb0ba746</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID2</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga48b43686cc64de8d233a4c6570661889</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PERIPH_ID3</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gad00b0b6f35df37248fb0c1dac166c035</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PCELL_ID0</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga75efbc491b08c94cf62dc2a111032c90</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PCELL_ID1</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gae1a623f6901860ed5c07ff99e66bbb79</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PCELL_ID2</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gac2510694ee55ad599b183c7b01cbd0f5</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_PCELL_ID3</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gabf4b1900107c50fd8b7705a7bcc0121b</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>gpio_mode</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga620d533a2ccc5296d2f6c8b95bf89fe1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_MODE_OUTPUT</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga620d533a2ccc5296d2f6c8b95bf89fe1aa5824cd687ac84449b093087c25cec7c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_MODE_INPUT</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga620d533a2ccc5296d2f6c8b95bf89fe1a61ddb59c5e9a9cc6d9cf99d11a36920b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_MODE_ANALOG</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga620d533a2ccc5296d2f6c8b95bf89fe1a5a5b8916bf45f2a4ff3818fa75f85d27</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>gpio_pullup</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga45e6c8e8730474e4644929cc5528400b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_PUPD_NONE</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga45e6c8e8730474e4644929cc5528400ba7807e2b9a6941fdb1326d9506af81a67</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_PUPD_PULLUP</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga45e6c8e8730474e4644929cc5528400ba10489125ed48a34dd001fa33e13ae396</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_PUPD_PULLDOWN</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga45e6c8e8730474e4644929cc5528400ba700268e334e769a26b44a8bf253017d2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>gpio_output_type</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gad59707c5e578193702cfb29b49b88108</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_OTYPE_PP</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggad59707c5e578193702cfb29b49b88108a6f3629cecfb427731be8487dce2970ef</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_OTYPE_OD</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggad59707c5e578193702cfb29b49b88108a2f6c5d8dcae60adb9f33fa337fad91ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>gpio_drive_strength</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga14eba84c1c8f80b08a770775d3bf060a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_DRIVE_2MA</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga14eba84c1c8f80b08a770775d3bf060aaea65a1945ac5ef1b9c311ce0795a136d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_DRIVE_4MA</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga14eba84c1c8f80b08a770775d3bf060aafea3985b235eba4f68685de32f0fd7a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_DRIVE_8MA</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga14eba84c1c8f80b08a770775d3bf060aa769a972b3ebbb6ad8dcf54abdb24df10</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_DRIVE_8MA_SLEW_CTL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gga14eba84c1c8f80b08a770775d3bf060aa13cff51bcb9a3b105a9d0c288bf73b83</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>gpio_trigger</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaf59a0ba6300bcfc24d4dbf13000a2fbe</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_TRIG_LVL_LOW</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggaf59a0ba6300bcfc24d4dbf13000a2fbeac94012614bbe9b2145034b62207cc52e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_TRIG_LVL_HIGH</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggaf59a0ba6300bcfc24d4dbf13000a2fbead8f2e505b5428ce834d192e911b87265</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_TRIG_EDGE_FALL</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggaf59a0ba6300bcfc24d4dbf13000a2fbea36956d1dafc73d361288ad3349b5ed19</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_TRIG_EDGE_RISE</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggaf59a0ba6300bcfc24d4dbf13000a2fbeae7e4da82de08018b2a2283404ad0bb91</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>GPIO_TRIG_EDGE_BOTH</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ggaf59a0ba6300bcfc24d4dbf13000a2fbea001639d96fae849f0c4672b265237da1</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_enable_ahb_aperture</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga973a4b2fdbe9c67f34d323e8426e45e3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_mode_setup</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga03c124c823e28cfee2bbf904ca12020b</anchor>
      <arglist>(uint32_t gpioport, enum gpio_mode mode, enum gpio_pullup pullup, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_set_output_config</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga83d7dc913be2bd27110019cae5ecf25d</anchor>
      <arglist>(uint32_t gpioport, enum gpio_output_type otype, enum gpio_drive_strength drive, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_set_af</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga08c5e974d93f738dcd80cf95b2447a6a</anchor>
      <arglist>(uint32_t gpioport, uint8_t alt_func_num, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_toggle</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gac7fe863cd889722d5e06916617b007b1</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_unlock_commit</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga070a8a3682a7e30e037a6de9b287355c</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_configure_trigger</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga6a3a20a7f47ef2eb7c6efa9360f51fc9</anchor>
      <arglist>(uint32_t gpioport, enum gpio_trigger trigger, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_enable_interrupts</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>ga6647e613abe8223c2e78f773dfa279ce</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_disable_interrupts</name>
      <anchorfile>group__gpio__defines.html</anchorfile>
      <anchor>gaa85cd2d1a8cbcfa7406e11afb15c1b6b</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_write</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga86b48f47f23889ffed56ab86c0605865</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios, uint8_t data)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_set</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga8ce0832cd5110fa7bbe0f6c968a04a09</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_clear</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>gaed9a0283ed78d8746f72a298889207f4</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint8_t</type>
      <name>gpio_port_read</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>gae3a2f79057d2a87a5f08b28242ea8ba8</anchor>
      <arglist>(uint32_t gpioport)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_port_write</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga00eeea592868471ce4eecce176b062f7</anchor>
      <arglist>(uint32_t gpioport, uint8_t data)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_clear_interrupt_flag</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga0eff5e23114c6346aff26c0b275d7464</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>gpio_reg_base</name>
    <title>GPIO register base addresses</title>
    <filename>group__gpio__reg__base.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>GPIOA</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>gac485358099728ddae050db37924dd6b7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOB</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga68b66ac73be4c836db878a42e1fea3cd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOC</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga2dca03332d620196ba943bc2346eaa08</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOD</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga7580b1a929ea9df59725ba9c18eba6ac</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOE</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>gae04bdb5e8acc47cab1d0532e6b0d0763</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOF</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga43c3022dede7c9db7a58d3c3409dbc8d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOG</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga02a2a23a32f9b02166a8c64012842414</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOH</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>gadeacbb43ae86c879945afe98c679b285</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOJ</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga7d3020a351195b6600a5d64c01c461fa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOK</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga273d78d198f0221223b3e9d7798f1649</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOL</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga845425770e695dff46b9b28c0e7ed90b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOM</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga3ea32fef9aa376b226f5c209bb0126b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPION</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>gafaa7a6254c4bdb2bc8170b77b4813233</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOP</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>gaccc0c6189ff0a9269a991eef22b0af58</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIOQ</name>
      <anchorfile>group__gpio__reg__base.html</anchorfile>
      <anchor>ga30c45ca759f238ca12afe1733a309fe6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>gpio_pin_id</name>
    <title>GPIO pin identifiers</title>
    <filename>group__gpio__pin__id.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>GPIO0</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga20f88dbc839eb32b5fec903474befdd7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO1</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>gabe59d3a7ce7a18e9440bd54cae1f3fc8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO2</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga88a95401ea8409c83cbda42f31450cd0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO3</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>gaf3cc04d651b622d5323d74dc2f0999a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO4</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga98aeff9c8b3bbdfd119e4ec4d3f615c8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO5</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga1a96368c99d63b0e715b7e0421f4a209</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO6</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga46027cd97ff756e5ddadcc10811b5699</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO7</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga3820cacb614277004870fc37b33ad084</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_ALL</name>
      <anchorfile>group__gpio__pin__id.html</anchorfile>
      <anchor>ga9f4da19f41fda0a3ec6d017e0bedfa4a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>nvic_defines</name>
    <title>Nested Vectored Interrupt Controller</title>
    <filename>group__nvic__defines.html</filename>
  </compound>
  <compound kind="group">
    <name>rcc_defines</name>
    <title>Reset and Clock Control</title>
    <filename>group__rcc__defines.html</filename>
    <member kind="enumeration">
      <type></type>
      <name>osc_src</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga1b534c93284a244cc1a84f41a459173a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>OSCSRC_MOSC</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga1b534c93284a244cc1a84f41a459173aadb9b675853b16e89501a2eb677c12894</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>OSCSRC_PIOSC</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga1b534c93284a244cc1a84f41a459173aa7168eb777903d908b7d91b952b3f1c58</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>OSCSRC_PIOSC_D4</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga1b534c93284a244cc1a84f41a459173aa24ecedc2dea27d6b5823bcce0133fa22</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>OSCSRC_30K_INT</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga1b534c93284a244cc1a84f41a459173aae39fab718557fa25a344a38c6d8469f3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>OSCSRC_32K_EXT</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga1b534c93284a244cc1a84f41a459173aac0bd6d9d0b5e0de9831d717589345c11</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>pwm_clkdiv</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga9634afd06a58256ecd18d15d3fed8e5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_2</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5ca15284be2c0bdfb7cfd1abc33586a0bce</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_4</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5ca192482b9872c838aa8439a7dc24872f3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_8</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5ca2b247289718884cea03bbf09cc6caf7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_16</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5ca5a9382f819f076e61ac36cbcb10f756c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_32</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5caf4549a3497ee484e5a6a2fc41a4cdb29</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>PWMDIV_64</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga9634afd06a58256ecd18d15d3fed8e5cab881e4379f382748ece5ec8fd5252b3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>xtal_t</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga8f09204cad649f070887682ed6549ea9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_4M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9aea78bea9d38f290229c93b114486dec9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_4M_096</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9af5a8c78ec3fa62fe9db636244d7aae2d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_4M_9152</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9aa9365b14fa1c2d02f3c03d644ecc7c5d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_5M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9ab0fce32c50d64cbd11cdeabe3c803a25</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_5M_12</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a06584b157ab517bfec8d48aa40d6134c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_6M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9aeb2a3ddd53e05557c3ffcea38adec4fe</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_6M_144</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9afcfaf7f8ca0ce1bc94fbb15e05d9c2cd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_7M_3728</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a0d4bc0fd92bd1e48594f5f19a3d38d3c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_8M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a76156a0243858d32154f20628fe06ea7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_8M_192</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a206a359c0b7effe7044e2036329b522c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_10M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9ac5ae1bca5c59cb91d93b693d123e7507</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_12M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a9ffa2eea00a874b497e48f28c9760920</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_12M_288</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9abb413dfd67aec6c42a74a4344371dedc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_13M_56</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a4364c57b346dfab730f7d5a8fc481343</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_14M_31818</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a4e82b3d2c5a44644ce20873733cabbfc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_16M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a72d6d58d8b24db9d249f76876e753016</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_16M_384</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a0712bb6a91be8e9bba76a804a267139b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_18M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a43224b6f62384fbdd777e598a2013a36</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_20M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a78ec47d31d8759b95ba7b939919ea68e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_24M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9abc98d766583e32076c0504b9297dfa8e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>XTAL_25M</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gga8f09204cad649f070887682ed6549ea9a8ac164b085138f1c24abb51ee35cf5d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_configure_xtal</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga2cd8f194ad903834c78212a0eeb05aa4</anchor>
      <arglist>(enum xtal_t xtal)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_disable_main_osc</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga5f536734177d63215d1494e6dea715d0</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_disable_interal_osc</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga25df4f03d0154ac5b09b875dad1226e0</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_main_osc</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gafc653bc7b616053515e1fd4aeb54f972</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_interal_osc</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga191546fa4ded6cb6d3c753e0de255464</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_rcc2</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gae166a674c8b6592adea0ff77c1e009e9</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_off</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga31217830e7f538e902e8b157e2715428</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_on</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gaf58b85261f65604132b67d3567c2b0c3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_osc_source</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga6acaa4f857ca454c3f90f579091b3246</anchor>
      <arglist>(enum osc_src src)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_bypass_disable</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gabbbe68ef690e48ae19d4ff04e69cc9c3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_bypass_enable</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga269054cc63981ae593e6820de2fe76b1</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_pll_divisor</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga104041e1468a0fc62d6c584a25859053</anchor>
      <arglist>(uint8_t div400)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_pwm_divisor</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gad9dea6bb566a5d6079037f41cc3d1c62</anchor>
      <arglist>(enum pwm_clkdiv div)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_usb_pll_off</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga68c7027c77c2a1a0d6d202e191c1baf5</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_usb_pll_on</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>gae31973474f6d00125a57784e84230c86</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_wait_for_pll_ready</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga0f5f3bea5dbfde10760a9cd775951a51</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_change_pll_divisor</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga70fca8e561f7e2b2b7062c22d85419b4</anchor>
      <arglist>(uint8_t plldiv400)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>rcc_get_system_clock_frequency</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga14ed842866228082443acad62da172f4</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_sysclk_config</name>
      <anchorfile>group__rcc__defines.html</anchorfile>
      <anchor>ga94e682bdf26bde662afb76403acdd227</anchor>
      <arglist>(enum osc_src src, enum xtal_t xtal, uint8_t pll_div400)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>ssi_defines</name>
    <title>Synchronous Serial Interface</title>
    <filename>group__ssi__defines.html</filename>
    <subgroup>ssi_base</subgroup>
    <member kind="define">
      <type>#define</type>
      <name>SSI_CR0</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga42848fe0c117f1ff719f3801d5aa2680</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_CR1</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga10464498ce11115ad90d7db3a9b343e1</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_DR</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>gaca41a444d99801dc07b1609434322289</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_SR</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga6dbaabaf26a3cf9f799e2aedda64a976</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_CPSR</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga6d711191bee46cd63ac6e008bbe89415</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_IM</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga36b5b43d0230a804949f729ff596c9f0</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_RIS</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>gacd096e24500cffbcaa3a7a80d5d38b29</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_MIS</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>gadeeb5ebb23eb805e39b7861f4c355d8d</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_ICR</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga70464021aa00aa847de064a816b4d9be</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_DMACTL</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga8c163597675e79c701f72032dfc2193e</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_CC</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga96f69900e8c3887c7dc82d831b5e8205</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID4</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga96471d945829658d9e03fb90ab43c937</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID5</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga5d39ef31f481f2df24abe6dc5e865c3c</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID6</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga51b58160ceb9ffc45d614313939d3e6d</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID7</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga063c1d8b96e1f8f71d6892d2a362a947</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID0</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga239887a62ecac0478facf58f7b2915f4</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID1</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga559c77aff03678ea9b88be82ba02d24c</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID2</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga6ad225fcc9ce118ca061cfd88cbfcade</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PERIPH_ID3</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga3be5ed28af28047f9df642d3020a1d03</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PCELL_ID0</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga92d02e0dbe52ba455b6f374d29a4d866</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PCELL_ID1</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>gac636b0d8ffdb95f3d285a8e82c976201</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PCELL_ID2</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>ga22c47d6c4a1316204db952454b270634</anchor>
      <arglist>(port)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI_PCELL_ID3</name>
      <anchorfile>group__ssi__defines.html</anchorfile>
      <anchor>gac92b2290be2869027821a1fc31c5a43e</anchor>
      <arglist>(port)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>ssi_base</name>
    <title>SSI register base addresses</title>
    <filename>group__ssi__base.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>SSI0</name>
      <anchorfile>group__ssi__base.html</anchorfile>
      <anchor>gac8ab5d11c2608b24b105c74da01088f4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI1</name>
      <anchorfile>group__ssi__base.html</anchorfile>
      <anchor>ga26e99cb407c039d9b5fa260e3a0c463a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI2</name>
      <anchorfile>group__ssi__base.html</anchorfile>
      <anchor>gaf7a5b340fc1e5948b228e5d13a2f0181</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SSI3</name>
      <anchorfile>group__ssi__base.html</anchorfile>
      <anchor>ga7fff594b0952c3f9ae26e62893661cae</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>systemcontrol_defines</name>
    <title>System Control</title>
    <filename>group__systemcontrol__defines.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5edf22a6ff060d03941c304b59f0356c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae38a424f0c881dab2d160d1e0a9a6131</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PBORCTL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga76a268a55ee06d4523cde5e981e25d00</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_LDORCTL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae1727f4e9282812be6f932cce51ddf65</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1c81f0f10746ec3948b6f46b4d06583f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga846407f0f364f2507891553773c77d19</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4f9f87f9381b3da165a82bde242d5e61</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab6b8c95db38367853b060f1c7fa53121</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga23e27fd6ce8421975c6d703068dc8fea</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLCFG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gafd79d5907c1916a809d7f7a4c840fd1e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4c4d9a120e8f86cd11fcf8e2af7b3104</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6a83bd485e529b5a78559157d5569755</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MOSCCTL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga25e07d3c958f6bac10d5a290d57bfae8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DSLPCLKCFG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga68c7187602ac287437278b7680d80ed1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SYSPROP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6daeba541534e298f90efd0f35b8b5c8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCCAL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga250323d022f46409e1d0f58a3e953f8a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCSTAT</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaccae2718c49e898e670929411106fddc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaafe0fae5b7bc00e01652773e2cb03d38</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga734cc69a16872dc5b0435dfc8e33ce8c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLSTAT</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga22dba56588d81f1db2a533a89128eb89</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1efbf63d9e8aff1b23bce262faef97db</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaadcd15bb3c76427ca5cc1527fc986c15</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3955755f8969bdb31e8a6120e3858947</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga894da3bc3acb2228e6885633818452b1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3c4439eec911da39692e2eca91ad133e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaafabd906ed5f891d6c2850ac476a8855</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga9d6cd0bb2d9ce4370183ccdf3a0d0f29</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4b3c3f57adcf2c1ad7078338ba1917bf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga9b4d83e2f9614143848a83cb2e98d45e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga2ed5cfe445620462380d37ae203af89b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga33d54fad389634e98f72b7d0c08aea27</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6628dce9c865dbd404cc128cfc4118b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae7f36b9ab7e2fd3407fc126a4de3cccb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1a632d7f025208b96eb7f076792f4b69</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPEEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabeabdb669aa33e8b4d567f04f6881739</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PPWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga11ffaae252e1fcdfe3b46d4b52069f97</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae52f07e6026cdffcfbb66c7e949254d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0c76a8ca571ad439072226ae778731d2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga2c0070136c26a60db90b69c8a36d55f1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga85fc6d04e544c769ef810f4b20335e62</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabc341b62d00a679132decfb16b08abb2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad1cc563458c06d54fd6ad1a005f5136d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga8957ae37f65450d5eeb330dbba058fbe</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gacb38bc82810ceb53602fbd9315d600bf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf6c69bc3b36566ab6b64e00bff1b2c8a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae0cf4514b7157ff3581048b8f4f7afae</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga96f2718fb61767be5ce86c5156b4be76</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga9cc6e115d1a61a43e522f8ee98c8cd42</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga9c75e336111c2f7be84e3d946070eaf5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga87306cd6fd517cdff582c1ac03317e06</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SREEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga39d9833a54e27edd54148e774f7f8140</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SRWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4e0f2b7801661f4b53ac3318ba0558be</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7fb6c282f04b8c0049a9a37b3c31ffde</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac2a17a27f27095d9183b01b62f4b6595</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga8af77153922a20e34a61458fe52a01a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga63b94e1ee057d847a3b743d2225b30c8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad18dd4000352df15f7747be4b2b26dd1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad6dab476e4bdc0c7a076aeb076824c7a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad9ab60781683a7d1034bc15d6580be9d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa89f93f8b0550662977d2042ca3763a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga2b4a22e34f60aa16590503bd28142e93</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5887eadb4604a6652da75d2da21704fe</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga370831e771886d3f1f55032fc3f68ac4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga939c17fa3fd86b64ff052438913e1852</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab690911dbceaab7546c4e29285215005</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga475582de09fc682400a2851e9487f84c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCEEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadb6f19b04e5d634d7b77050c9974cd00</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCGCWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gace339e2ae7d3ef98917903d02f56867d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gafececbf0e6767bb73907652f4fe917b1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga21e1a44ea9d264cb32e3635022a2df7e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaebf3e2e2e2282562e8a5db6e2523f4a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae538e8bc54cd976ce5fc19175077a417</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac5c80a3adce209fa73b22b617dc3c4ef</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab775e111e8f836f33fe305991d248a09</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab153bbf9505acdf02e6edb0a7c628370</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga32248274bb68b0db5373fd906c87e1f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6968f26f5c99c0bae49a8dc8cab49c26</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gacb97dbf5e9f92b919853903369364329</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa3c720bebe058ac6cc7509f01b6af037</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga747164418de6ef193383a654d6928a21</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gacccb6cd8b29fbbbc4b992d9b0cbabdc5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf0cc1ea315d6f1d229c44995d513098b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCEEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7276e913a291bc59d4fe4567df87374e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SCGCWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0e8aee59e053bd372ce3489621d7b2e0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3feb68760536b91d6692fb33f852b0a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga2d041d9c90f92c684c4a94c671ae6a7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga28485c0da92d2cfe63d28828acc9d738</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae95030f6b3efd410b411ac692b91b125</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae6881bf890b30baf274a7612f8bc6ef5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7a752c2bea315fb46115e1e26ccd01a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabb9d58b230ec70a87d1cce3bb4881e34</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0e80aa32ef1f2784611f99e0272d47f0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga40dad8c00b7c7da2edc386fb22cef20a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabe1b1a21bc6e8d51d14720473baa7498</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab5ac513e510f39d285a9450e3edb3036</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga755efcbb2b5fbfbbec6a7b0bd0f617e0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga161b9a5692a946c4e523d0e2af5e3768</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga33ce68cc9b4d5d1166336e6ef3399529</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCEEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga280dc3868cf374a65e7e20d5a94d6cda</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DCGCWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6b22c80cb897cc0217e0315e7d555475</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRWD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga900c50df903fd51991022c9cc9f7076c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab858a6231af82b2258787ef5cfe53732</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRGPIO</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf1bed14a3a8eed561aea3e5c84c40b9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRDMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0837b1f4fab2e15dd731cf2d1606c498</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRHIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac07b587d02a1ace8b61991a6f30842a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRUART</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac949c8408d49d794e49ba2b16441b7f1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRSSI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3ec3fed2ee8b70aa37db0ae2841aa8d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRI2C</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae4c8cb028475676d356e3ef9a07dcdc3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRUSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae70bb6bf8e20657594f5b6373e193a55</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRCAN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab48eb50560f5b6b458be977397714f49</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRADC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaba3f1792eb395a0f8b38d4c961e2a59d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRACMP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa8ba78c3d6d3a656e5a138c4d5db056e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRPWM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac5755edacae296f8e5d4627e1315a057</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRQEI</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaba3ff5073c7fc9d81a5ffdbe432efd41</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PREEPROM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga72036e113c08ca9ae659b53783fd05ee</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PRWTIMER</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6486d5d92ea650b07334969a7ab5841f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID0_VER_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga16fab30229e50e97c454779527ff21fb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID0_CLASS_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf8bb02a59247d7b08c480087ab2e1480</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID0_MAJOR_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga668bea5191a35cf5ba22de0b725db302</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID0_MAJOR_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga668bea5191a35cf5ba22de0b725db302</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_VER_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadf0fa38d20bea1cf67cc0afda5a1f633</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_FAM_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga66b84de779d7b3dbdb4554b4f9dd60b0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PARTNO_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga75bb789203b3904316be5bbe9b135ffa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga116bad021d6f0688c725d02fe0f59e09</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_28P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadec1e38c0b22d94e2ee49ea99bc9ba0a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_48P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga943ae94331b490333036170ab4543efa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_100P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa586a1208e6ebcb6a4ff98d94c187779</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_64P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga918446ea5313f4a9e416b7879df560fd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_144P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga946f0fc4a6c5d065fa07d5250379ae4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PINCOUNT_157P</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga63a97e645822b8f4eefffd9e26c7ad69</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_TEMP_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaefcdc20b79db7ea39dca801de7d05bcc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_TEMP_0_70</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaeb5fe6056323d0a8c56139245cd26a43</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_TEMP_M40_85</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7773b3919bfdf6277a09b685e45f361c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_TEMP_M40_105</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac4dd7e8b70faa2e9665be17be9f09353</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PKG_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga9c8296b6274230daca4a92527594e967</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PKG_SOIC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab126cf34cf0698a1bcd5045438b1a9fc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PKG_LQFP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga213d722ec1c0fa19739e73e2ccf74b34</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_PKG_BGA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1883c4b05238910aebe405d2b58a3db0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_ROHS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad0ae95623f9e30aea72f842faf4be39e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_DID1_QUAL_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga29a41f5c7cec39ba357c62b73e2aa3cd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PBORCTL_BORIOR</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga17be2a80a5f12b1f3adb947808fc9a53</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS_MOSCPUPRIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab0bfa3bae579b53620e5c32eeea9754c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS_USBPLLLRIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gafbff7e2f4937563ef5396053afc6713a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS_PLLLRIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab4b46aa0c168f00095a5a4994467c539</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS_MOFRIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga043a18036c8a16fa3c72fc2823af8a5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RIS_BORRIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga8830434fff930be690343b1a506c5802</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC_MOSCPUPIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0e9e13a620e765e7736850ab7679eefc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC_USBPLLLIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga18f7c58233d8b834658aa8a7588b1316</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC_PLLLIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf73977c87d091aa6159296f472745bf1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC_MOFIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga54fa62dfa94ad6fe4cf33584b92b1840</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_IMC_BORIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1e2b48e339478299a6d07294ca3979bf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC_MOSCPUPMIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga39b53fa64655b23d0a0fdbf9f604dd46</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC_USBPLLLMIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0d75024828b61b8570be7a518d0d308a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC_PLLLMIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga393020f45f93446f69823444302de1b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC_MOFMIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1cd11060075cd940baa520848f9092d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MISC_BORMIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab3ca5b7ac46ab12145d4f7f2d29cc0bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_MOSCFAIL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga78a489a3461f0a1030166f2eb457b01a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_WDT1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5259680ab0eede32935d6b6ee12744f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_SW</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae8083764645fb2f1c454121a9cd6c280</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_WDT0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae5ef0182c55f62f02947303b53f6faea</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_BOR</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga09aa727dc2aff547d05a0acd47b2887a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_POR</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3e84c488a026bab3831a04c55f3b7f27</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RESC_EXT</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4ac589004611f905c614ba65329f91ca</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_ACG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5590c014f49535187c545a5ab327e6a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_SYSDIV_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5ba712f7625b543eb4888ded74a4a869</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_USESYSDIV</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf8fb495f4cdd46774964a5f822087001</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_USEPWMDIV</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga10e7f6f0d034bae2138721e90fc9dd6b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5e15a75863a2b288387662de01f59e84</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga01f48838c712ddf4b86b76682258e3bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6ebed8830a1a8a604eca53cff3bc4ef8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_8</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga129ecea0a47ab0c30f54f004b154db07</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_16</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad4fd684b00e4409765dbd296762536d7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_32</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga20916fc545dbab9a4ad2a8a9b16578d2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWMDIV_64</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4f38be712c8610d138b05e8e92b56520</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_PWRDN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga35905e3ba9779f862e9e0e6c33061158</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_BYPASS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga97ff3e281b18a61b1d23718154d69110</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1e3c5cee1ded72a0214bf73697b6cea2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_4M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad4c98ea4884c22d748c5e8c4f5c78d93</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_4M_096</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga8b539118e530ad56b93d999b7bfbfb38</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_4M_9152</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga18b7aaadd8cba93246074510129219a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_5M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7448a6a094aaef1fd7f75728ede64b52</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_5M_12</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gacdd0dcc9611ee726742f9da5036cff5d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_6M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0062596e239b082956255a032b32e91c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_6M_144</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga11b18c90f9ff662fcff003375adb4936</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_7M_3728</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac3231b318f4c4f9ea6838eee6d9df126</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_8M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga40fe004c9131158bd830593432d88041</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_8M_192</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadfec5bed87d78fdf699e58f22ceef265</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_10M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga84a40c104aefdb2756f3e120bb2ec874</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_12M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga1f0e9a28a8eb6e8190d0f62bc0ce5179</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_12M_288</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gac9f64c7d381dca626345e272e922c89d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_13M_56</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gafdef4b66c2f1d81cae7face37596ddcf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_14M_31818</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga10e2c183d4ac23267ffe080f3b485082</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_16M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabbe8f1289943564b54d1edb99387406d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_16M_384</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7fba322dac3a69e304e11f46ed8bfbcf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_18M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae27885cd48447c40c66e41b9df8e123e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_20M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaaab9e55393bbacbca286a367c60037d9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_24M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5ab501f6af5ec4d42c884c7608a3e37c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_XTAL_25M</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga014129309ae8bfea4cb997ad49b85fff</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_OSCSRC_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadaba83b7fd53775cb6545334f638afb3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_OSCSRC_MOSC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad28d62e26fce970839bcc6157cd35440</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_OSCSRC_PIOSC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5277826f37c52e1ffa794139cb747eb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_OSCSRC_PIOSC_D4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf9cf675e83698fb7c04947091811427d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_OSCSRC_30K</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf1861fb0e7919a84216ae2950d4f719a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_IOSCDIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaabd0bfee458c4b7f2f483ec08fa9c82d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC_MOSCDIS</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3d2bb3a1bc4e46017c5c8f25185e3ed3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTQ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaebc0ec7e19ad5accb7fe77d661016799</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6397fcdd0d640d0ebfeed87686903eaf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga65d9811ffe09003d97d0e539dc45902f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaf35a041a06e9f9a93d89226121723bf6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabc26d305bf549408317368f84465d181</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4b0379eca5a5a92d72acc564aa3e2e00</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTJ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gadbd5f1e4f6e99dbbe0c659fcb6011dc9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTH</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga25493559f5191ab7e90a452be6a0d5c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga621b3c71764bcb53c53110e222f7e9ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTF</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga0483090ce2140b5db668459ae294d12e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTE</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga17d2d81d71284849b077d6e446ebc06a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6c2eda9f1a6327b6a8231658c669bda7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gad23f76581152dc78675508920b334e52</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga30e677036aef32db26aae7297b732baa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_GPIOHBCTL_PORTA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gacbf33723e49cff0a0748381de5933880</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_USERCC2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae5f20734a9ca83207a16e5ef75318940</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_DIV400</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa06ac42bcfd26bc1d713eba6f7acf74d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_ACG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga09f717daf244a1efff38b95a2eef83bf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_SYSDIV2_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6de1a25f37d60bb4f2f49a4b691a6f83</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_SYSDIV2LSB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga22f13ac333a74a868044424d01f853b3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_SYSDIV400_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga89b6d9fb176ef85e127497218a3a91bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_USBPWRDN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga27e58030fca66cf089e0fd0229c0db28</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_PWRDN2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5aeb661402986a55ada899546102f7df</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_BYPASS2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga00ba5e9742453395621589a080457f63</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gafb523b9664fb8539080b41106d194a89</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_MOSC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3a0072b73139dcfa4e086e4e3d8aca16</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_PIOSC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5c6ce2235aef567e8ea9feda8a8996e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_PIOSC_D4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga7be4e4e29831cd4fbf146d72ee84fa39</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_30K</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6e1303766e78da4401a59ab99daec0ec</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_RCC2_OSCSRC2_32K768</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaec6a1781103cdfe45b390d6856f75a97</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MOSCCTL_NOXTAL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaad290593a7a5690ac2f0515a1ef83cb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MOSCCTL_MOSCIM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa7a5cb84236f5c8c176ed467c578dbd2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_MOSCCTL_CVAL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga092e760a4bc66d3544f29ff175df9366</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_SYSPROP_FPU</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga23fe88bea7d69df255daedf476353743</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCCAL_UTEN</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gabe8883f93985822cf049113037a37158</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCCAL_CAL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4b7490f14b79ecf8591ed125410c3b91</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCCAL_UPDATE</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5cae7d8761f0d7acde6a1a5ca285d3e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCCAL_UT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gab0fe6557190dc884a37bc89884e122d6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCSTAT_DT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga4a35717735468af3790f3a4526cdc2c1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCSTAT_RESULT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa44363e26b73e3ad79a09860e812505f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PIOSCSTAT_CT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga5c01b10085ffc992f64dc59512ea6d7e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ0_MFRAC_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gae2e13fc92489891e24595b8cdb2231de</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ0_MINT_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3e441aca630218f74f104b5039180335</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ1_Q_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga6db7f0f8f99685158a3678015c839a92</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLFREQ1_N_MASK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga19fb32e61c40610b8c9beb55ed808946</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYSCTL_PLLSTAT_LOCK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga3b453f1ca6c5b4cb11e25d7639fb7999</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>lm4f_clken</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga216f3c694b4491151311f37062d17d7c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WD0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca4ed4b9759029811b3e6649cd52b061e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WD1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3da71e1dd5031a1ef729ae46b721442a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa6ee7be0a28b7b0863ffa3036d6348d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca41d26b0f1d712fd1b5e4907ec6c003bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca0cbcdf4908b061573cc0c4755c7a63cf</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8cd23ab393b1e48df121992f01aeda8b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca2ebf4a17f114d0755d56c70270296856</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_TIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca742bb71633532784defb27bc08929131</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3679ad09af8988bc42fbb91d3f4dc680</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cabe0e8ee9db293583511579bffd694b6c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caca99d40a98c0cccaa8dd4a5eb8422dea</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cade83c8caa918ccda0766791076460d1e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOE</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caecccb0767e9ec411349051aa5f055779</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOF</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac2a10a31a77eac6a9e5ef9b59397abf4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5db55f45d23bd84c2e4b9a36c7cdb57e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOH</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca600bd6f61da0eaed2c2f8fdf49fbb9c9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOJ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca102cef7c361f0abf6978c4f8d0dc9fdf</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca4d57a22df2e2953d15d9d6cf17452368</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca348b6ea39fc7fa98a87655fdc6a502c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa58410e8cbf0f1d4225ee31127bd3cc2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPION</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac669e50f84467592773b50cb4939b1db</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caded2683ae4d5dc85648ce717abed5b4f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_GPIOQ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca58f20fdf421eb45995d87dcfbacd7167</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_DMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3ae8e2842eb2835ad420db68142cb4d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_HIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cacadf3729d36aff283a6b995c50d07a9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa9acb07259a107f34fb9bc1465318358</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5114880810642497fb718a780cb97199</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7e1da0d80c6c33928981ad95122fbb4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab8b4af332d6573bfa11f678c012a6a30</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cacddaf98763a8dc6edf0b69c983becfd7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca0d7a994107ab9af33f9bdbda74ec4685</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART6</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac1d0755185366f087db203361eee488c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_UART7</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab404d28f5b889694df6aacd5c5ee204b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_SSI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca26b4e2ab32dc0600d8b4c0a8446b2ce4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_SSI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca0ea40540a8ae621bd6b037313f141b01</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_SSI2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6c202eaad688d9b50a10dba5849167af</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_SSI3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca0deb692bbeba86e74a775a6421dc0575</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8811f698225b6f22675835384d7cdbc5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7611e5aa9f71a5ecd6f466dc090533f0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caba410e2e71ca9f59b76176672c9b1e4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caead627335ffc92260faac70ef409eff7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca84f22df0eedbb2ba90bc26bcb7d4da8b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_I2C5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac6e02b50494884bcceba6b24a9c26778</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_USB0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5363c94212e95049b036c487eff3e1d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_CAN0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab58916255b8946d66b0cf6a7c7ee18ae</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_CAN1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca61b240aefa01d614e06844f5b5943dc9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_ADC0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cae2455d48a21d53fe277cd3c5adf24d0c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_ADC1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca81b9534841bd60ff16455a28481d7a14</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_ACMP0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab043d91156620faf65bd2e7ba3b4892f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_PWM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5f8590f1bd8c06757c2ec264263d681a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_PWM1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca170fd02a576cd3e58beba221c63b995e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_QEI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caeacbf3016e47c9eb8042554666b3bcad</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_QEI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca0d25bd38b9d39e34504c068a85a66c45</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_EEPROM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8b25367eed70bf66750893969395d9c4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6383fd9f173243f9c552c63443a28f02</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca359d9e637ee81970457b6f2ad8ecf612</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca43c053c8a36d6ced3001bbc1181aa31e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6f8b1e2b9dce0bff8e294821af562aab</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caffb993127d5ad7ff814afd0b5e41bb67</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RCC_WTIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab7be18f91861ada479d141a71cdc4a73</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WD0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca046cc93f0c75e6a9f3c8d82296dbedc3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WD1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caedd62d0f4aeabc04c335f753ddd16675</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caab752e13fea766e909c1efb6d369af83</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6b6ae62b91785b7844c09efa72dba2d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caee4143c3631243bb89674442f966c8ac</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5dc46082bc7ebb37c3a0335a6e94aa13</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7caff6329167c3bdab800fb2a0d07122</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_TIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cabd8109243e642ce6d837c9f6f850f395</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca44f76e63809a5df30f158bb8cbfcb4af</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cad2438eef51dd7790d4faac2e461f61bc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab212b98035b8c1d7a52c66b8a8753ea7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1646b5369fa9b52ffe45deb801dffe53</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOE</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa4f1793bd6ab343073e151f470113a98</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOF</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac2f5731fc83d86f8f850c984c769ed97</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cadb087b1ddece696b0c76a7fc03e44de7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOH</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3869bb86c68604dc9015445cad5aa173</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOJ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6db1e5291f26eac23359ac9dae6d950c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cae33b8aad16ccb33ebba1d1eaa6f855f5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca87ce63fcd19271797660eee6b705d786</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca57d107799f9336f31e84b0cadd455e72</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPION</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca11a7252fd2351f7e48825fa6b209280b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caad3a48f5b917e478e276d137c0854433</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_GPIOQ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cade40950fd79bd4be332fd1a5a86620d9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_DMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa5659c38a81a48fd717ebde84b64aea8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_HIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cafa505d83b1672bacbab31c68d3b892d4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca96b4af86f966ab2d37b95ddcfd7ccae0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf0d58b6788b09b9b25ef36b180af6005</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca507b539a5e8127d359ff57e5c546a3e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1cafdc841e97ef1e45fac9f77cd67bfd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca27200fdd011075f80ec18eac21d811a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf2fdcaa5d35ac4f63fc12563ec64665d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART6</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa423f043367f931e13636b18fa6ce8fc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_UART7</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caeecfc516a3a6732cc6f05d38d66b9cfb</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_SSI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1b7f56125e323df352ae5391bb40e386</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_SSI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca2316ac607983f5bafef7f42aa821705b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_SSI2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa61fed56149618b50ddb926328a3e90e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_SSI3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca4fb5b1a1e53833917ba9862b11d85c44</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa8da3fc9fc64bc9bbbb2c0bd5c207ae0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caac1edecb6b1268334dd61d0be70eb8b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6e133432144b494871b9fc7434a91214</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cad1c9598c70b899240aa22be7b66e4765</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca866e650d47a300b010a0d6cf92986ef0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_I2C5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca22435962f8a22e2843a4fdcde07e8bb2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_USB0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3e2c89fb8b1a25854efa6173e3abbd60</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_CAN0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cafcc5638cfddfd30f1c4f0b72e7621784</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_CAN1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cace6a0c67861a7f9063364e3c23996b7c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_ADC0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8aeea8da59ca1b091f5ffe1fbe03350b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_ADC1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5000d6ca5e3b40a5865485dd8dac5ab8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_ACMP0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6d58998c9f779236ff93a78c89d20f92</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_PWM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab5b9111da3ae69f232b705e8bbeccc1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_PWM1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca53d2cc961160d1e6c05fa4e180656c29</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_QEI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca90dc05d2e2deb1bf75bd052269d9daac</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_QEI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caab3ed767192a9f1c572958d99e36422f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_EEPROM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3358317c6870dd5d3f344534992faf20</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caaa5749f1284d0fd60e24dceed3e19845</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca21e28d01f0c4aeb1c57918a47419419e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1b1772a391965c6c63491a4cc2199f33</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa6d69371c0aefa29ad28c84a8d2c5e1b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cae949847f800769a79798400b6276bc3b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SCC_WTIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf497645a2e76034377d94db5bffda572</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WD0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca66a2b58595f766c88a89456d92a6999b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WD1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf27b9bc9fbc7f398886d53051e1c7e9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf4e4e5d577b31124e1439065df8d0388</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca05457312f35ee02c99445845bd774cf5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cad3d86893051956ff70634657615333dd</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca344f155fcccd30402c20051ea1aff24e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca451f2738d3868dcf745fe9a32eff995e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_TIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6bd98b42f2e9d897f9e2c30557fd3de9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab780a6472576a549c8282bd72424f6ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca463b8efb0b1d18397145221619849d10</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOC</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caf93e5bd2886d5750ba8448f6def7f7e2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOD</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1e3b60573f8876b42812cd2533f43afe</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOE</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca052e00cabf7b5f4f879de58d1b800a34</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOF</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caeb39f15184175728c1f66ae9b2fc6368</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOG</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca6a007341f0ec22ced18048a8d3dcee5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOH</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca32edc749039153efea9aec9cd73b5c57</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOJ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caeec160671808558a02f0fef5cf4421f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOK</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7cb603574124a5b4cad0dd4761082173</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOL</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cae06811c44ba8daf345c72fafe861996e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOM</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab52675509b66c5eb36a859554102f26c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPION</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca77191f653c4b3a8200759aebf37113a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOP</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca503607a6c996f27fe0dd7ca5b13a1090</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_GPIOQ</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caefac3afd723c7499b7ee6bc024edcb1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_DMA</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cafdafb51e56845ffca6b8715f9f034be9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_HIB</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca10d534586668d3d60f4082a7324a7bd8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca1e3076c6ebdab342bc02a25e1496c2ee</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caaab6521fa757e01bc08531474b332f2b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca41de1c88f35ceb3c07d35f671ad53508</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa73e22c8fcf45b72716dc6e19c3182a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab4aa5c7ee2f1d2001da02150b22d62c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca5b30d06ea2c9c48b0550b7f48be0214a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART6</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca70e3a920c0cecaf2b5d17b8c9640c979</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_UART7</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cacd88c770eafbb36b4dfe11aec93eb29f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_SSI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca648f5b18454e1f34fe2b5f7a26787c1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_SSI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cafecbd79096060a7d5719ffd7fee65416</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_SSI2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac415fc33e675e3a0c7b9c83e283d8324</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_SSI3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca3003b45b36dd2c510d9e7d2450701f39</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caee1726dc7502226128f256d5888c3fe7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca230e1ad541a3279fbda81dbf6d160a3c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca790e86296a06d23c43f629410741ec17</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab8c14dd3cc4990cb7c078208458a850d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7caa5980152513b981d5257c21194e256e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_I2C5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca56f9aef0fa4a67fe5e73db5e251818c1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_USB0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8f353ad79c2bae6065a14d0912dde03c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_CAN0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cafa65ea4c7879f7b4362f380cab768981</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_CAN1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cad12005b67225647d17685c1f4d5172d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_ADC0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cad5ab3be3260e7f2063c5565f63509d28</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_ADC1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7aff55fefdd807626eed54b113497898</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_ACMP0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac174433556444d8b81c462697d008a43</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_PWM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca7579e619e6f15718ce4632ae0c5002ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_PWM1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca9bfbba89e407bf5acb591a071df5bd43</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_QEI0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cabb79eacfc68137024a6dc3f7756e8f0a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_QEI1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cac0b3e282db4a0066e7a770c6f0449d56</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_EEPROM0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7cab1db2619cdcfb45c70fd49196b179961</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER0</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca215fbe3cc93bb7378d5c1c3fc30f8463</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER1</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca86630e306040d178a3f981156ad3db0c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER2</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca18eb7b7308cfbfe58ad6905cfabceeaa</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER3</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca8923e4b18bf6c1df5d744a022f87dbf2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER4</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca4049b8d01dbe4fe6ad17d4af74ad4757</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DCC_WTIMER5</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gga216f3c694b4491151311f37062d17d7ca43d42bf96f91a44c4fb69aa9b21e7bad</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>periph_clock_enable</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>ga82ac341aee840ff3b2fc15d79de016c9</anchor>
      <arglist>(enum lm4f_clken periph)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>periph_clock_disable</name>
      <anchorfile>group__systemcontrol__defines.html</anchorfile>
      <anchor>gaa8ca86965f943013fd3bc6eb61f889e2</anchor>
      <arglist>(enum lm4f_clken periph)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>uart_defines</name>
    <title>UART Control</title>
    <filename>group__uart__defines.html</filename>
    <subgroup>uart_reg_base</subgroup>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0c98d6b342b60683f8ab1be7fc4cfe67</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RSR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga22b6ff089fceba4538995f696d674f93</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_ECR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4455df69f03245da7e55e1e6f66e9f51</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gafa91f42e4c05178702d91ab5a044ed3f</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_ILPR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae2882a6a4b8b962ed7d13c57ef61418c</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IBRD</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga56c90494604bf4236327dfc604a70bec</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FBRD</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gac2699a574273c40349ce203d84ea098c</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gafb713a577cf19294540b99d005239012</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga481e29e3dd902b257304a54765c631d7</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaa97170d41c922b22fbcee85d846d9723</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaf303f95e542d800b839da69a164b48fd</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RIS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gafa4d3c48ac3fb9156570f0d3869be6d6</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_MIS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaedd0d984219cde25cc9bd4c98df53edb</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_ICR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga386e0b9ad35c2153352035406c95bfce</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DMACTL</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2df54bef23a4d6b5f04911898c05d615</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaa015d4be34d27b7efedfa70c2bc47dd9</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LSS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga5033a846ffa37746dd420f0abbd192f5</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LTIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gace0ee38f677ca6fb5e087ee5f9471387</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_9BITADDR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab9be99b9a61ea4ec1971ec04aaa92647</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_9BITAMASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga59bcdcef048c7038e01d209eff6871ab</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PP</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4d571827104805cbcff1546b4597e060</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CC</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga65cb0afad62b63adee375cb2100d8078</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3a626dd544a0c862c8d4a05cb6fa0568</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID5</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga07274a1430cf6020433edb13d4efb69d</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID6</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga94a1b773bf4e75927c4c4c5379057d00</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID7</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga30fcf1da9dab8ef0b33e9937cecd020e</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID0</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab3214df5e67cc4704e9eee9ebd95f3fc</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID1</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga828f5a604381369427c2f38f0c928151</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad27b8abe99a87eee9991ff7f9eca6d3d</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PERIPH_ID3</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae012a9706c5138fd2fb48622ff1d1379</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PCELL_ID0</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga9978829c0bd2ff2ecf4804e2e34b9b07</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PCELL_ID1</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga444cae1f835a85f653b23116fda02630</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PCELL_ID2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga222ae22f0a5052fc95be975d08763140</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_PCELL_ID3</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8b9b032313b6d2299152397c9cd6be91</anchor>
      <arglist>(uart_base)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR_OE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaddc33d9ca903b5498498845fedcc2406</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR_BE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8abd67385293e64f5736e5faddc68909</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR_PE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3ce57f5f6c7670322e73a4156223a03c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR_FE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga759524d95ccfeb42ae6973ef1e727e92</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DR_DATA_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6e9ca2dfc3ca7f53c6c217784c0f298a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RSR_OE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga7d66e764b50faba0d5327e912b6d85a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RSR_BE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga59aa21bf1e8bbec3739106f17e956188</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RSR_PE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae017de851d1d1433b4b968b74ed4446b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_RSR_FE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad9663ec3a20819ba90fc7c8e6a3c17cf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_TXFE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga08ea2055746abf83b7336ae08dd1c92d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_RXFF</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga842ff6a0449123ede0b5b93425ce902c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_TXFF</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2f6a08ae8a3005e737005cbd607081b1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_RXFE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga9ba067e6425a6c5b5aca79874c549364</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_BUSY</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga39a3e9403d1914dba75ca838fdc73364</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_FR_CTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3f708d80e15117726f4faf915fc8c349</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_SPS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga44c6291aea20dfcb8d75fc9d47c1ee10</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_WLEN_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2402827931937dadf399741d30b83e09</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_WLEN_5</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2b26de9efce73ee468f7060ef685bf85</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_WLEN_6</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6458da85c7d1c15e3f1b6a70893ab906</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_WLEN_7</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3c312edc8c30df376cbb7a702d799c12</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_WLEN_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae0e27af519dfbffe6d6a50942bdf30f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_FEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gac63fc7bdadb98a24125de76e1468510e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_STP2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3aa637fc03bb39a175932d19c48e2e5b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_EPS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaff1a8a6c54483dbf84d1902397d47b7b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_PEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaf1d5a9b375d10c260bf2e7b85bcbfe0b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCRH_BRK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga80da31de1b31f9d4cd25906e43a9a919</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_CTSEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga25c8b58bfb4a36de3f897c2863888856</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_RTSEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga75a515e952135c324c71b86c1c39991c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_RTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gabfa4cdce464b8d60caafc85a577cc2d4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_DTR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga64e1cffe33d4ea911bf3f32bf6baf584</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_RXE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4d522022557e403572e518db25b3cf5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_TXE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gac81859db681e3918f88e0f7aea596a06</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_LBE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad6ac64f4d5b8d6377bfd1d3799813710</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_LIN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gafe830fab8bf36f567e24c9b09f8660df</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_HSE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0e25d885a9f3a9afed6cae6cbda20273</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_EOT</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga94ba876fdc237d15d664750dc8035264</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_SMART</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad1df1c7e40e3fb26b09e99f3a15105f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_SIRLIP</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaec5956fa3d9f55bb1345fdb0cb7b7800</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_SIREN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga5b9bcd2c0a0180f3a61c1aa0afa36697</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CTL_UARTEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3d170d515d0d3082a4a4720c2a8c4fde</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0f71f525c36b1043fa2c3c724de52799</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_1_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad35fd9fe2144098c33ed0e6fea59bf59</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_1_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8ad9b3a77cdd5bfc377e56417351334a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_1_2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8bdfe54997f9d323be44777226a43379</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_3_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga625f6e1af03073f7a09874c9586e34f6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_RXIFLSEL_7_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaca1bedd6d2af97b2f54337365cfcc575</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3f91489b140ae6f2548b356066403b03</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_7_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaced1ed4db0e884f3baa7c4bc6bec37e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_3_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6e6afe4ba438f753fe8af611ceb5b225</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_1_2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4fe2390470f14c8fcb46696182667908</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_1_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8bf3308a27ce97a489c78f1568362082</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IFLS_TXIFLSEL_1_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga81f70b7961b17d393bfe0c96d725291c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_LME5IM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3b11e7cfa01c185335cb158f34db2ba3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_LME1IM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gabde8ea82682a67057ad6541d737c9451</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_LMSBIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaedf14b595013f396376ba168660967f2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_9BITIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae7a05ad05d5e3ba626aa112f6572a83c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_OEIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga31eb54dcaa6a8b8f0cfa9b2546ecf90a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_BEIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga46f192668edb0a97e140d59d561f8018</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_PEIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga82fb24baaca8bb73091c345efce113ba</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_FEIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaa1bc63532f94fa71e77b820c211642e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_RTIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga527c259858bfb707366288c0469c38e2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_TXIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2a390d47a4075123384c44717159d4bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_RXIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gabad98dd5a214f571cb0dca49383fed17</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_DSRIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4c491dc6be46ec71bbffff12bdcedc7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_DCDIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga3c41556d8182126dbaa32460313e5448</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_CTSIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaff361cbb8eb14b4b8e7af5038ec6675b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_IM_RIIM</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gacb7bd7884ad0aa579942724b83678bbf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DMACTL_DMAERR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab1fd3871951b17e306c5e616f0c131a0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DMACTL_TXDMAE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga237d4a08c2819e668fb7d7fb5ad308cd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_DMACTL_RXDMAE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad01fa53a8abced4fbc821b3c34289d1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_BLEN_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0891c8cc859b3f8077cd08a6c47b846a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_BLEN_16T</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab30fd25f25590abb6c2cb1e1fc56cca9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_BLEN_15T</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad8da29741fc4d2fbf008b38ea94287a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_BLEN_14T</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga4bf662defab18af97f5973c232ec08f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_BLEN_13T</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga79a1ceca660d027fe4743a290816c0f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_LCTL_MASTER</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gae23b5e9084af31d3c231a4adf4b5a0ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_UART_9BITADDR_9BITEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6626602aac55324657ffc813e648929f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_UART_9BITADDR_ADDR_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaec1129baa25b7a59e3b310414f00cdb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_UART_PP_NB</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga5d450e8bec95b40e6cb5b9fa3c5c251b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_UART_PP_SC</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6f426020f7f410b2cc85a8fd17814189</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CC_CS_MASK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaaeadf2fb733578296ea3699644173773</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CC_CS_SYSCLK</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gafc8eff880490901a538ee6ea23364d18</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART_CC_CS_PIOSC</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaa7e2d80dd63bb6eb2dd73ffae92fa6bc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>uart_parity</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaef0010d952178bd0d851bcf3b63daeca</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_PARITY_NONE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaef0010d952178bd0d851bcf3b63daecaaa80d2d8ea61454045ebe71d155e85b3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_PARITY_ODD</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaef0010d952178bd0d851bcf3b63daecaad90cc425f5ba447773a44a75be6593e2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_PARITY_EVEN</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaef0010d952178bd0d851bcf3b63daecaad908a637b1dd23f93b149dd2d8bdfdb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_PARITY_STICK_0</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaef0010d952178bd0d851bcf3b63daecaa38e4b2e7c5b38024a01f2195748b77f3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_PARITY_STICK_1</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaef0010d952178bd0d851bcf3b63daecaaf20f92778e29fd61d30d276c03353085</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>uart_flowctl</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga50eb7d8c1338b309a11dda5e86240706</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FLOWCTL_NONE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gga50eb7d8c1338b309a11dda5e86240706a087e42c5229a10b6b02aa7f3548e0e18</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FLOWCTL_RTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gga50eb7d8c1338b309a11dda5e86240706a8105549da2b48f557f98e5575dd48fc5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FLOWCTL_CTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gga50eb7d8c1338b309a11dda5e86240706a0d9da2df46dc2f980c019b5c0c955a2c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FLOWCTL_RTS_CTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gga50eb7d8c1338b309a11dda5e86240706a8e06515da93c430fa99ac8838772040e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>uart_interrupt_flag</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad02c1455d23262a6dbda84d6053d1f7e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_LME5</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea22d9d80af21c49fd2845e65ae8964dfc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_LME1</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eabea77aada0f38984ab084b876d26e23c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_LMSB</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea1c2452c3b449ba88c7796845815a974d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_9BIT</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea8775b5683b20ec15d876196c580c584a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_OE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea9de058b4403afc1fa2aa7099c45d0b06</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_BE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea163453765b5cb996248cd1145c1dac7b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_PE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eafcfec516eb93a5484263c358cfe27fe9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_FE</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eaf222be4a67ad2277a89bca5696d17fbb</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_RT</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eab0ad539c76928c0113725547575376ca</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_TX</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea2a145ce488762550b48d111c76e03a60</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_RX</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea32eca039b291092af8df630b6ea6613d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_DSR</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eab03ddef3181bfb7cb0e11bfbb8a42950</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_DCD</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea26117cec3a3d09416668f45aad1ae08f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_CTS</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7eac8355e07c854f7e22921e01bbc17d5df</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_INT_RI</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggad02c1455d23262a6dbda84d6053d1f7ea19236a41b0921ff78c01940fb02a9654</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>uart_fifo_rx_trigger_level</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gabb9ce81df1867ef09b554e02c63f9be5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_RX_TRIG_1_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggabb9ce81df1867ef09b554e02c63f9be5a9a9d22fe9ad81f42dc7dd48e3c0522f9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_RX_TRIG_1_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggabb9ce81df1867ef09b554e02c63f9be5a698200c109ea3fb1035cc81ff5c3f34c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_RX_TRIG_1_2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggabb9ce81df1867ef09b554e02c63f9be5a370cf31f3500cdbd65b9d6acd71efc2e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_RX_TRIG_3_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggabb9ce81df1867ef09b554e02c63f9be5a4f86835a14f9d693d8982534bc2e50e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_RX_TRIG_7_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggabb9ce81df1867ef09b554e02c63f9be5a73fd9cb5457a327d74fdf727be6f580e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>uart_fifo_tx_trigger_level</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaf79e87f1a6f23cc0501f4d2e3667a848</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_TX_TRIG_7_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaf79e87f1a6f23cc0501f4d2e3667a848ad73b67cad4b485679cd70f2af3fa73f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_TX_TRIG_3_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaf79e87f1a6f23cc0501f4d2e3667a848a0ba87eebf5636a89499d1c75f4524fc5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_TX_TRIG_1_2</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaf79e87f1a6f23cc0501f4d2e3667a848a6cf07bc1412bbf702f39ad478add22fc</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_TX_TRIG_1_4</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaf79e87f1a6f23cc0501f4d2e3667a848a3b6765a42c9e8794949062283819221a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>UART_FIFO_TX_TRIG_1_8</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ggaf79e87f1a6f23cc0501f4d2e3667a848a38c34c00f7ccd0a9424cf3d48338a4c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_baudrate</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga05a4513fed24167dd33ebc5805b9ca77</anchor>
      <arglist>(uint32_t uart, uint32_t baud)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_databits</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gadafb5a1c4596fb687bdc99dff191d88f</anchor>
      <arglist>(uint32_t uart, uint8_t databits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_stopbits</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaf3ddb4b6b173fb6cd30139ac67ca62cb</anchor>
      <arglist>(uint32_t uart, uint8_t stopbits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_parity</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga7b0acac78989b94dab26d92d1b640e05</anchor>
      <arglist>(uint32_t uart, enum uart_parity parity)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_mode</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga35e0dc6cb6910a92c425b6303006e273</anchor>
      <arglist>(uint32_t uart, uint32_t mode)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_flow_control</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gabc37af12e01b5827d40f5ed7d149076f</anchor>
      <arglist>(uint32_t uart, enum uart_flowctl flow)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gacd6ffe5e9e3f555d07a3af7ee7fdf74a</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0167757699a14269e55640b5034ffcbf</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clock_from_piosc</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab9d2f5223bc6f08621e9b6d38f8a4fb9</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clock_from_sysclk</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga917d75f562fe93eea7c684756f9f02c1</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_send</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga225444640f9510f126a597004fe3dcfc</anchor>
      <arglist>(uint32_t uart, uint16_t data)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>uart_recv</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga58e1b434c54c8994d9120194e2b542ab</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_wait_send_ready</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga9546f62d7bb1695a34748ae418d2ed7f</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_wait_recv_ready</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga42ff0fbee0d1b80cc77667c14c3213dc</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_send_blocking</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2a621068c90e83ee0d9646235dda7ba6</anchor>
      <arglist>(uint32_t uart, uint16_t data)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>uart_recv_blocking</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga5a7498f353526e8cb9e6c8bc5735d1f1</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_rx_dma</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga1028a66c1d0794aa79c9f9c3f116f124</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_rx_dma</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga0260bfb6563f9ffde298d76e71265994</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_tx_dma</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga8efe31511341bd42f7ed5c60aa4a29a2</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_tx_dma</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga92c430ac58b1716402b67ec2ab3be426</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_fifo</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga01026f3f8ddd0939fd8e291ea3837365</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_fifo</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gad0ef308eb0c6b5a6009c97b3127d014c</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_fifo_trigger_levels</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga37814e8f7102ff91e9e73d17ffb0e55a</anchor>
      <arglist>(uint32_t uart, enum uart_fifo_rx_trigger_level rx_level, enum uart_fifo_tx_trigger_level tx_level)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_interrupts</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga20cf32aae3969a9b1e857cd1ca145a27</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_interrupts</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2fdf35d91a90e2452368b0358d942f93</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_rx_interrupt</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gac40fe4cd2b2d331d83100e305c2f6f78</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_rx_interrupt</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga6c46b4228c1da4010f8e8d5a66bde0c0</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_tx_interrupt</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gaab2f52c4fdb9b8ba9e89670e93727ecf</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_tx_interrupt</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>ga2d6559da0d940412e88374af5fe8a5dc</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clear_interrupt_flag</name>
      <anchorfile>group__uart__defines.html</anchorfile>
      <anchor>gab5fd7ee81bc609e4aca33a4b9079611b</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_tx_fifo_empty</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>gad1246f6b0b156022d942ded6a7a7e398</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_rx_fifo_full</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga6a2584c859c685b3ff9693c7e9b1d657</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_rx_fifo_empty</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga840603de8127f2b63abdd1f5d113f938</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>uart_reg_base</name>
    <title>UART register base addresses</title>
    <filename>group__uart__reg__base.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>UART0</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga0508661f121639ffdee7de2353a0def2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART1</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga8d69bf04d07af4fbbab5a8bd291f65ff</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART2</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga7f6bd6eb89ae2eeae97af4207ebe3cde</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART3</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga961726a611b38bcaf61f3d598b0a59ec</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART4</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga7c035f6f443c999fc043b2b7fb598800</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART5</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga9274e37cf5e8a174fc5dd627b98ec0fe</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART6</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga61531d783c55be06875e7029331ffd67</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>UART7</name>
      <anchorfile>group__uart__reg__base.html</anchorfile>
      <anchor>ga20bc10f5b73e8b51724b2f23c5b2e785</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_defines</name>
    <title>USB Controller</title>
    <filename>group__usb__defines.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>USB_FADDR</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad92d008d03f5d8f09f16181e9e52e876</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaef1a9dc21299c653d5abf0dc1a27a502</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXIS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga03e4c678a0ddae17ca23e85e65f26814</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXIS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf2c9fffa60198a23c882e7ddf8549d83</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXIE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga743c579f79e4fa19802f53437e1e1ca7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXIE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga5ff4ac77ee784c979df2860a64b8d106</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gafffdb3b9440c9a4e26ff84ff533f573b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab4e58ccbade58db72ac11981737ec51d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FRAME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa4f674d9fa6558a47c8f89b2a45b49b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EPIDX</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga5950b670d8ea2f0a93627f84fdcb402a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TEST</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga64956051c35bf0ef75d23c974697e078</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFO8</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga38333c938233a8c2ad9d3c19f3bb1e2d</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFO16</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga6f48d071c29984bd3bdb53f92189c14f</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFO32</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga6492869d57f9cfd220a782ff7833ba0a</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXFIFOSZ</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf5861a491d3a9671ed07dbc2db38e781</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXFIFOSZ</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaed8243b4826430be9c4e3bee9f8429c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXFIFOADD</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga96d11af3bcb3340682b9dfee288c8c09</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXFIFOADD</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad961db7bb51ee9ff79b5edb6d641c622</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CONTIM</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga0fbe0d83e408acf19c0f304081e71483</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FSEOF</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab15e63a3693b6703c87b9b2adffdd2b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_LSEOF</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gac116c270b960020a0a37f42b5cc32115</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga6ef57c5869b4b001a49f327c1d33f166</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRH0</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga576c2ea5c5dd89ae4752036e63722a05</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_COUNT0</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf7ecfad56454fbf1f8192323bdced3aa</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXMAXP</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf0769e1a61a5341db94173a0acd332c8</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf324f8096f3711bdb9580b294157dc60</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga27de1c7931c1f9ea6021fe6581ac66c6</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXMAXP</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga7b0b108a19aaca9923d40c5801dcbac2</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga50c1e6369d5fdb2971a17abaa5d1e8b5</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga1b512c91287f77cb42345427b8143ace</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCOUNT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga3ad7f9f4d763a4736ce92aa59eb590b8</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXDPKTBUFDIS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gac3f01ef4204c55f6c827476bf53dc354</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXDPKTBUFDIS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga1ebfcf49d3be483454d870c6044c335d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRRIS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga43be47e6b25da4e00a0f9a42f33e69eb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRIM</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga85292bcffaea0579b45c5c8202da7ac3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRISC</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga12db7a2f100c32770a3507dc7b35cc88</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DMASEL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gafce8c301e416593b42c5ffbf0b430563</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab123b4a1a9f145fcd93c9612044cf585</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FADDR_FUNCADDR_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga2d13500deda4ef0d8907b4a1eb004da1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_ISOUP</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf571b356b470125bebacd49b2a9ca638</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_SOFTCONN</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga924294c1d15dc9e28ed111eea2e24afc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_RESET</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga07be6b6ae6bc5b8829d50f4364e72170</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga1b7d36d2650b70ee144894419244e4c8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_SUSPEND</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga26f24c171021e851e56f6bc09ce30e1f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_POWER_PWRDNPHY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga173a03da0b3497bbce7301e0a41c06f5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP7</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga58c090fa4a5e2d9963d163f7fe18e732</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP6</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaaed43c78cabd4bf8b55914cbf7942335</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP5</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga3614760777b40aa0f2125acf4fbefc5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP4</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga60874fded11c6c5d491a5e47ce185ea3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP3</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad2dd7082012d04a39d15368605a7b072</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP2</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga487de857d562aec62e81e9bd0ab11ad4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP1</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga8ff7e6f3fb1426d9d0bb2eeb4ff20e4e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EP0</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga7148f9b5726dedfd6a36f52fdb9ce66a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IM_DISCON</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga7d16c57db8f5544d983924d2234dc79a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IM_SOF</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga9b6c0d1dc4e39220ec26dcde2f174441</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IM_RESET</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga988539e74865cb5e79c49ab0acc30684</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IM_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4d82a47e24cbcfe0a89c9aa961b0048d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_IM_SUSPEND</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gabca1735f53faf4a44593ef12e1c2bfc3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FRAME_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga9631e2d739ae0d4b9dfab5dbe331a49d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_EPIDX_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gac9a64b85a8fd5eb11a5f7f4585883f66</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TEST_FIFOACC</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga8ef6bcfd81ebb6bd35155adcc311755a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TEST_FORCEFS</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga55a48b6aae7b23e5dd88dc7f169b33ef</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_DPB</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf21c7990149621025fcec87431139718</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaddc4a3f6756a16013069a4e95b097de0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_8</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4c564ad6cb10c62d559eee1c1a9c9bd6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_16</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga994491866569641e236d75a626d9c591</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_32</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga835a73b30026bb2f107303a9204b4b20</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_64</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga21636367bc81b16e1fba5a827ebb446b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_128</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab8708d0c9dc528de2f5e9f797d1f4b5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_256</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4b7bb7ca245ba2a1dcb413b0bda30060</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_512</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa3c29579f391a7692b6cbbd6a0772a51</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_1024</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad8e6120f36226fe9c6bfa1f8efa7ecf6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_FIFOSZ_SIZE_2048</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf7774421e9038191ea20b0ef6fd2d5c7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CONTIM_WTCON_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab505befaaba2947c414d796373aecbc0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CONTIM_WTID_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga79c97c91a179d566af79a876c15f4424</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_SETENDC</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf99ad022830386b6eeec1668cf509228</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_RXRDYC</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga48e2f3edca2d034139b6ee968e041d34</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_STALL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab41d8813224b450b8d5fb8602d7f8362</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_SETEND</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gab649bf662d9310d93032aadde771d022</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_DATAEND</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga9d0b019991becfdfb91fe4227daad63e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_STALLED</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa0f1b7e9ac27a773a386af520e89980d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_TXRDY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga66cb7a914c02dce1716cb0b6669da104</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRL0_RXRDY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa46f20c660cc47f9b3d8f9feace51914</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_CSRH0_FLUSH</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gae6749e9d7f8d7b2afaa90cd3fa1e50ad</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_CLRDT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga9193bb8711914d2e9dde021851048bdd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_STALLED</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaba32c0c3e09bae0fbe4c95175f739775</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_STALL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad77a762d47d7e430638ef7766ba9c73c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_FLUSH</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga6da8bfded6fba86ebdc00cc81392988f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_UNDRN</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad8d302684eac7340cd649f65e6ffb72a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_FIFONE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4f699992cf30339abe4b680d77c98698</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRL_TXRDY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4bba6046fa89a27bc00833c00f4ead3e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_AUTOSET</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga663fce4e5169fad7ef3db84585a96faf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_ISO</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaac2f13d9dea99747a60ff3627d0e6e04</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_MODE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga4b6f1deb46bd6ea8273fac808e11d008</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_DMAEN</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga10b8ec666f75c0efd120c63639046a01</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_FDT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga5a9c4bd6b01cd885d6a50d4ee7a3d2a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_TXCSRH_DMAMOD</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaad543c87d547913611c0c11327c1d7a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_CLRDT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga096ac7b32487adc81e7148b608b5aa7c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_STALLED</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga66c90350c2da3cac2f95a1af1bc00efe</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_STALL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga1ad21c3fd83c33cc42724833ef965597</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_FLUSH</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga21bbfb545360dca03a05fa70a6e060ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_DATAERR</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga2bb6163609f40338cf48f6cfa1e7f0f1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_OVER</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa5bc944309abd867cee26f41d4fd014b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_FULL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf2cbaaa37b7bd4220dd808434178d3bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRL_RXRDY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga3642595bc567347f2de41412c8f788be</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH_AUTOCL</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga031c4b2c98827bbce50f5f25d3863ce6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH_ISO</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga850bc5c228e3200419bf97ac080161f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH_DMAEN</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga772c7bfe52b1a13810d9a685f50ed0ec</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH_PIDERR</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gae2513db8a7ef5237a5b38245a5e909b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_RXCSRH_DMAMOD</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaa259ba84966b3ad1925e7ef571effbb3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRRIS_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga64968f0eaacb51f2ba574ae7691dff83</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRIM_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga94d7abb9de415ca2387b3dffb10a4fe0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_DRISC_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gacb49a37729f80d7ca8d7f9b241096522</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_ECNT_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga09a4247eecdb0bd2ffd01d9d7d89551f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_USB_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga9e33ae0ff5e70604f711bc6f391524ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_USB_NA</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf2ebff7652c2fe18b88f28dde7337c68</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_USB_DEVICE</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga227db2fe354d138410c90000895bdb97</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_USB_HOST</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga1bc6803dbdc5917c5f8958a00b4cbe91</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_USB_OTG</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga28caf357c58dc1fd14a751eb37591754</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_PHY</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga54656f02a50d670f35ea6fc707ce956d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>USB_PP_TYPE_MASK</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga7a5b770f09b525fb6cd581f5c87035b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>usb_interrupt</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga0f2f22f2ab06463da36ae9c6c2a63ea6</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_INT_DISCON</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gga0f2f22f2ab06463da36ae9c6c2a63ea6ab7203dd6b18353416e992310394ed995</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_INT_SOF</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gga0f2f22f2ab06463da36ae9c6c2a63ea6a0d3be5f3ef316460b344e80ead93a4e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_INT_RESET</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gga0f2f22f2ab06463da36ae9c6c2a63ea6aa45692bc816bdd9bf6819db51f98ad3a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_INT_RESUME</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gga0f2f22f2ab06463da36ae9c6c2a63ea6a3d7fca27fa1e3edbc72a0b278398ca42</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_INT_SUSPEND</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gga0f2f22f2ab06463da36ae9c6c2a63ea6aa36cce94b72cb32846cb6478806e933b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>usb_ep_interrupt</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gaf0a19719081410c9b09041a7dd65cd11</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP0_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11a93a6bce8bca9bc79fb14552c840ae41d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP1_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11a90bb9c7bfad2314c174df49c83129a3c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP2_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11a4325207f3d1ac8680292e80fe4361c51</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP3_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11ae7eeedf54e147181ba987a244951154e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP4_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11a188f2d830cb08f1eaa8ca64d6fa90510</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP5_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11ae49ed0b247c7d8c63f13818096e5218f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP6_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11ab578ebf1ba7080c3e865681a91da0192</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_EP7_INT</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ggaf0a19719081410c9b09041a7dd65cd11a9735ada9074d7afcc7fbaee751b8e6a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usb_enable_interrupts</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>gad6acfc8a14795d10ae71053b0cfbf717</anchor>
      <arglist>(enum usb_interrupt ints, enum usb_ep_interrupt rx_ints, enum usb_ep_interrupt tx_ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usb_disable_interrupts</name>
      <anchorfile>group__usb__defines.html</anchorfile>
      <anchor>ga86efee4380227664681caba4d4c2a010</anchor>
      <arglist>(enum usb_interrupt ints, enum usb_ep_interrupt rx_ints, enum usb_ep_interrupt tx_ints)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>gpio_file</name>
    <title>GPIO</title>
    <filename>group__gpio__file.html</filename>
    <subgroup>gpio_config</subgroup>
    <subgroup>gpio_control</subgroup>
    <subgroup>gpio_irq</subgroup>
    <member kind="define">
      <type>#define</type>
      <name>GPIO_LOCK_UNLOCK_CODE</name>
      <anchorfile>group__gpio__file.html</anchorfile>
      <anchor>ga7b4058c05682f371e3c210e272a2d003</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>gpio_config</name>
    <title>GPIO pin configuration</title>
    <filename>group__gpio__config.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>PCTL_AF</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>gada5265d9b7ca84756a8876b34ae5a65c</anchor>
      <arglist>(pin, af)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>PCTL_MASK</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga7af7fa835329184da3a5a53bd4ce6a72</anchor>
      <arglist>(pin)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_enable_ahb_aperture</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga973a4b2fdbe9c67f34d323e8426e45e3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_mode_setup</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga03c124c823e28cfee2bbf904ca12020b</anchor>
      <arglist>(uint32_t gpioport, enum gpio_mode mode, enum gpio_pullup pullup, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_set_output_config</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga83d7dc913be2bd27110019cae5ecf25d</anchor>
      <arglist>(uint32_t gpioport, enum gpio_output_type otype, enum gpio_drive_strength drive, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_set_af</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga08c5e974d93f738dcd80cf95b2447a6a</anchor>
      <arglist>(uint32_t gpioport, uint8_t alt_func_num, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_unlock_commit</name>
      <anchorfile>group__gpio__config.html</anchorfile>
      <anchor>ga070a8a3682a7e30e037a6de9b287355c</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <docanchor file="group__gpio__config" title="Enabling GPIO ports">gpio_api_enable</docanchor>
    <docanchor file="group__gpio__config" title="Configuring pins as inputs">gpio_api_in</docanchor>
    <docanchor file="group__gpio__config" title="Configuring pins as outputs">gpio_api_out</docanchor>
    <docanchor file="group__gpio__config" title="Configuring pins as analog function">gpio_api_analog</docanchor>
    <docanchor file="group__gpio__config" title="Configuring pins as alternate functions">gpio_api_alf_func</docanchor>
    <docanchor file="group__gpio__config" title="Changing configuration of special function pins">gpio_api_sfpins</docanchor>
  </compound>
  <compound kind="group">
    <name>gpio_control</name>
    <title>GPIO pin control</title>
    <filename>group__gpio__control.html</filename>
    <member kind="function">
      <type>void</type>
      <name>gpio_toggle</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>gac7fe863cd889722d5e06916617b007b1</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint8_t</type>
      <name>gpio_read</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga4efb268c94f062228de2339d8273af82</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_write</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga86b48f47f23889ffed56ab86c0605865</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios, uint8_t data)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_set</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga8ce0832cd5110fa7bbe0f6c968a04a09</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_clear</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>gaed9a0283ed78d8746f72a298889207f4</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint8_t</type>
      <name>gpio_port_read</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>gae3a2f79057d2a87a5f08b28242ea8ba8</anchor>
      <arglist>(uint32_t gpioport)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_port_write</name>
      <anchorfile>group__gpio__control.html</anchorfile>
      <anchor>ga00eeea592868471ce4eecce176b062f7</anchor>
      <arglist>(uint32_t gpioport, uint8_t data)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>gpio_irq</name>
    <title>GPIO Interrupt control</title>
    <filename>group__gpio__irq.html</filename>
    <member kind="function">
      <type>void</type>
      <name>gpio_configure_trigger</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga6a3a20a7f47ef2eb7c6efa9360f51fc9</anchor>
      <arglist>(uint32_t gpioport, enum gpio_trigger trigger, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_enable_interrupts</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga6647e613abe8223c2e78f773dfa279ce</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>gpio_disable_interrupts</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>gaa85cd2d1a8cbcfa7406e11afb15c1b6b</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>gpio_is_interrupt_source</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga6526582457c0936b5a978b2548336610</anchor>
      <arglist>(uint32_t gpioport, uint8_t srcpins)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>gpio_clear_interrupt_flag</name>
      <anchorfile>group__gpio__irq.html</anchorfile>
      <anchor>ga0eff5e23114c6346aff26c0b275d7464</anchor>
      <arglist>(uint32_t gpioport, uint8_t gpios)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>rcc_file</name>
    <title>RCC</title>
    <filename>group__rcc__file.html</filename>
    <subgroup>rcc_low_level</subgroup>
    <subgroup>rcc_high_level</subgroup>
  </compound>
  <compound kind="group">
    <name>rcc_low_level</name>
    <title>Low-level clock control API</title>
    <filename>group__rcc__low__level.html</filename>
    <member kind="function">
      <type>void</type>
      <name>rcc_configure_xtal</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga2cd8f194ad903834c78212a0eeb05aa4</anchor>
      <arglist>(enum xtal_t xtal)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_disable_main_osc</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga5f536734177d63215d1494e6dea715d0</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_disable_interal_osc</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga25df4f03d0154ac5b09b875dad1226e0</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_main_osc</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gafc653bc7b616053515e1fd4aeb54f972</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_interal_osc</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga191546fa4ded6cb6d3c753e0de255464</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_enable_rcc2</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gae166a674c8b6592adea0ff77c1e009e9</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_off</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga31217830e7f538e902e8b157e2715428</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_on</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gaf58b85261f65604132b67d3567c2b0c3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_osc_source</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga6acaa4f857ca454c3f90f579091b3246</anchor>
      <arglist>(enum osc_src src)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_bypass_disable</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gabbbe68ef690e48ae19d4ff04e69cc9c3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_pll_bypass_enable</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga269054cc63981ae593e6820de2fe76b1</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_pll_divisor</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga104041e1468a0fc62d6c584a25859053</anchor>
      <arglist>(uint8_t div400)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_set_pwm_divisor</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gad9dea6bb566a5d6079037f41cc3d1c62</anchor>
      <arglist>(enum pwm_clkdiv div)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_usb_pll_off</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga68c7027c77c2a1a0d6d202e191c1baf5</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_usb_pll_on</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>gae31973474f6d00125a57784e84230c86</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_wait_for_pll_ready</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga0f5f3bea5dbfde10760a9cd775951a51</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>lm4f_rcc_sysclk_freq</name>
      <anchorfile>group__rcc__low__level.html</anchorfile>
      <anchor>ga53588dcb636da3e059dd7c45d82f6504</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>rcc_high_level</name>
    <title>High-level clock control API</title>
    <filename>group__rcc__high__level.html</filename>
    <member kind="function">
      <type>void</type>
      <name>rcc_change_pll_divisor</name>
      <anchorfile>group__rcc__high__level.html</anchorfile>
      <anchor>ga1c5a8dbbc0a6bac380b0041962075269</anchor>
      <arglist>(uint8_t pll_div400)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>rcc_get_system_clock_frequency</name>
      <anchorfile>group__rcc__high__level.html</anchorfile>
      <anchor>ga14ed842866228082443acad62da172f4</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint32_t</type>
      <name>xtal_to_freq</name>
      <anchorfile>group__rcc__high__level.html</anchorfile>
      <anchor>ga43a00aa2241426ed96573abc3de839e9</anchor>
      <arglist>(enum xtal_t xtal)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rcc_sysclk_config</name>
      <anchorfile>group__rcc__high__level.html</anchorfile>
      <anchor>ga94e682bdf26bde662afb76403acdd227</anchor>
      <arglist>(enum osc_src src, enum xtal_t xtal, uint8_t pll_div400)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>uart_file</name>
    <title>UART</title>
    <filename>group__uart__file.html</filename>
    <subgroup>uart_config</subgroup>
    <subgroup>uart_send_recv</subgroup>
    <subgroup>uart_irq</subgroup>
    <subgroup>uart_dma</subgroup>
    <subgroup>uart_fifo</subgroup>
  </compound>
  <compound kind="group">
    <name>uart_config</name>
    <title>UART configuration</title>
    <filename>group__uart__config.html</filename>
    <member kind="function">
      <type>void</type>
      <name>uart_enable</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>gacd6ffe5e9e3f555d07a3af7ee7fdf74a</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>ga0167757699a14269e55640b5034ffcbf</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_baudrate</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>ga05a4513fed24167dd33ebc5805b9ca77</anchor>
      <arglist>(uint32_t uart, uint32_t baud)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_databits</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>gadafb5a1c4596fb687bdc99dff191d88f</anchor>
      <arglist>(uint32_t uart, uint8_t databits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_stopbits</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>gaf3ddb4b6b173fb6cd30139ac67ca62cb</anchor>
      <arglist>(uint32_t uart, uint8_t stopbits)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_parity</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>ga7b0acac78989b94dab26d92d1b640e05</anchor>
      <arglist>(uint32_t uart, enum uart_parity parity)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_flow_control</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>gabc37af12e01b5827d40f5ed7d149076f</anchor>
      <arglist>(uint32_t uart, enum uart_flowctl flow)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clock_from_piosc</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>gab9d2f5223bc6f08621e9b6d38f8a4fb9</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clock_from_sysclk</name>
      <anchorfile>group__uart__config.html</anchorfile>
      <anchor>ga917d75f562fe93eea7c684756f9f02c1</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>uart_send_recv</name>
    <title>UART transmission and reception</title>
    <filename>group__uart__send__recv.html</filename>
    <member kind="function">
      <type>void</type>
      <name>uart_send</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga225444640f9510f126a597004fe3dcfc</anchor>
      <arglist>(uint32_t uart, uint16_t data)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>uart_recv</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga58e1b434c54c8994d9120194e2b542ab</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_wait_send_ready</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga9546f62d7bb1695a34748ae418d2ed7f</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_wait_recv_ready</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga42ff0fbee0d1b80cc77667c14c3213dc</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_send_blocking</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga2a621068c90e83ee0d9646235dda7ba6</anchor>
      <arglist>(uint32_t uart, uint16_t data)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>uart_recv_blocking</name>
      <anchorfile>group__uart__send__recv.html</anchorfile>
      <anchor>ga5a7498f353526e8cb9e6c8bc5735d1f1</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>uart_irq</name>
    <title>UART Interrupt control</title>
    <filename>group__uart__irq.html</filename>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_interrupts</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga20cf32aae3969a9b1e857cd1ca145a27</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_interrupts</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga2fdf35d91a90e2452368b0358d942f93</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_rx_interrupt</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>gac40fe4cd2b2d331d83100e305c2f6f78</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_rx_interrupt</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga6c46b4228c1da4010f8e8d5a66bde0c0</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_tx_interrupt</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>gaab2f52c4fdb9b8ba9e89670e93727ecf</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_tx_interrupt</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga2d6559da0d940412e88374af5fe8a5dc</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_clear_interrupt_flag</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>gab5fd7ee81bc609e4aca33a4b9079611b</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag ints)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_interrupt_source</name>
      <anchorfile>group__uart__irq.html</anchorfile>
      <anchor>ga6271114e5fe89f29dd5243c7c0c1934d</anchor>
      <arglist>(uint32_t uart, enum uart_interrupt_flag source)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>uart_dma</name>
    <title>UART DMA control</title>
    <filename>group__uart__dma.html</filename>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_rx_dma</name>
      <anchorfile>group__uart__dma.html</anchorfile>
      <anchor>ga1028a66c1d0794aa79c9f9c3f116f124</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_rx_dma</name>
      <anchorfile>group__uart__dma.html</anchorfile>
      <anchor>ga0260bfb6563f9ffde298d76e71265994</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_tx_dma</name>
      <anchorfile>group__uart__dma.html</anchorfile>
      <anchor>ga8efe31511341bd42f7ed5c60aa4a29a2</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_tx_dma</name>
      <anchorfile>group__uart__dma.html</anchorfile>
      <anchor>ga92c430ac58b1716402b67ec2ab3be426</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>uart_fifo</name>
    <title>UART FIFO control</title>
    <filename>group__uart__fifo.html</filename>
    <member kind="function">
      <type>void</type>
      <name>uart_enable_fifo</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga01026f3f8ddd0939fd8e291ea3837365</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_disable_fifo</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>gad0ef308eb0c6b5a6009c97b3127d014c</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>uart_set_fifo_trigger_levels</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga37814e8f7102ff91e9e73d17ffb0e55a</anchor>
      <arglist>(uint32_t uart, enum uart_fifo_rx_trigger_level rx_level, enum uart_fifo_tx_trigger_level tx_level)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_tx_fifo_full</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga509d7ccd81a766a5a240558f85e3ca0b</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_tx_fifo_empty</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>gad1246f6b0b156022d942ded6a7a7e398</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_rx_fifo_full</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga6a2584c859c685b3ff9693c7e9b1d657</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>uart_is_rx_fifo_empty</name>
      <anchorfile>group__uart__fifo.html</anchorfile>
      <anchor>ga840603de8127f2b63abdd1f5d113f938</anchor>
      <arglist>(uint32_t uart)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_file</name>
    <title>USB</title>
    <filename>group__usb__file.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>MAX_FIFO_RAM</name>
      <anchorfile>group__usb__file.html</anchorfile>
      <anchor>gad5f9b20acf4cf9b527ae4463b4940971</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usb_enable_interrupts</name>
      <anchorfile>group__usb__file.html</anchorfile>
      <anchor>gad6acfc8a14795d10ae71053b0cfbf717</anchor>
      <arglist>(enum usb_interrupt ints, enum usb_ep_interrupt rx_ints, enum usb_ep_interrupt tx_ints)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usb_disable_interrupts</name>
      <anchorfile>group__usb__file.html</anchorfile>
      <anchor>ga86efee4380227664681caba4d4c2a010</anchor>
      <arglist>(enum usb_interrupt ints, enum usb_ep_interrupt rx_ints, enum usb_ep_interrupt tx_ints)</arglist>
    </member>
    <member kind="variable">
      <type>const struct _usbd_driver</type>
      <name>lm4f_usb_driver</name>
      <anchorfile>group__usb__file.html</anchorfile>
      <anchor>gaedb3ad0d80317ac7e69d6e616077df06</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="page">
    <name>lgpl_license</name>
    <title>libopencm3 License</title>
    <filename>lgpl_license</filename>
  </compound>
  <compound kind="page">
    <name>index</name>
    <title>libopencm3 LM4F</title>
    <filename>index</filename>
  </compound>
</tagfile>

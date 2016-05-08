<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>assert.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>assert_8h</filename>
    <includes id="common_8h" name="common.h" local="no" imported="no">libopencm3/cm3/common.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>CM3_LIKELY</name>
      <anchorfile>group__debugging.html</anchorfile>
      <anchor>ga2b90b6f44540e6e706864903ddc35926</anchor>
      <arglist>(expr)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>cm3_assert</name>
      <anchorfile>group__debugging.html</anchorfile>
      <anchor>gaac76fc08d5852b8bdcf33b5e6448355e</anchor>
      <arglist>(expr)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>cm3_assert_not_reached</name>
      <anchorfile>group__debugging.html</anchorfile>
      <anchor>gac2ec555ba39f6c80aa9f3a9289864076</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cm3_assert_failed</name>
      <anchorfile>group__debugging.html</anchorfile>
      <anchor>ga9c0fa84fe9c0e99ad57a3d079ba41ddc</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cm3_assert_failed_verbose</name>
      <anchorfile>group__debugging.html</anchorfile>
      <anchor>ga93d8d0dfa33f2bc46e3fa201813ef3bc</anchor>
      <arglist>(const char *file, int line, const char *func, const char *assert_expr)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>common.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>common_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>BEGIN_DECLS</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>acd011fce71bdd0f1884aa638d921487a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>END_DECLS</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a67019e6c1b6d267f6f85fbb577f0f286</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>LIBOPENCM3_DEPRECATED</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>ac323d74304fff7c24906846e6b079642</anchor>
      <arglist>(x)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>MMIO8</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a90fa73df23b66cce82b8633dd05e2b02</anchor>
      <arglist>(addr)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>MMIO16</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>abb0189383a963a89a7607442c420ba3f</anchor>
      <arglist>(addr)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>MMIO32</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>aa424823d5a675828feee4e8be0a64a65</anchor>
      <arglist>(addr)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>MMIO64</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>ac297642f26c83beb456c4aae0556baab</anchor>
      <arglist>(addr)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BBIO_SRAM</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a91288ffebd7cc75b3ffef37a1908f903</anchor>
      <arglist>(addr, bit)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BBIO_PERIPH</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>ababcd1043a94143801dae779d1559c4c</anchor>
      <arglist>(addr, bit)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT0</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>ad4d43f8748b542bce39e18790f845ecc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT1</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a601923eba46784638244c1ebf2622a2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT2</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a9c9560bccccb00174801c728f1ed1399</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT3</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a8e44574a8a8becc885b05f3bc367ef6a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT4</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>aa731e0b6cf75f4e637ee88959315f5e4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT5</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>ae692bc3df48028ceb1ddc2534a993bb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT6</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>acc2d074401e2b6322ee8f03476c24677</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT7</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>aa6b8f3261ae9e2e1043380c192f7b5f0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT8</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a0e80e65237843fa1ff15c68cd78066f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT9</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a3aa20ab5eb33383fa31b0e94f4401cdf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT10</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a8c0f01fdf020d0f7467449b181fe95cb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT11</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a2cda1debde057b596766eba6a76daca0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT12</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>aaa0a6acba8436baabcaa1e91fad6c0bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT13</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a965dc1748ab1cf91426bd04a2fe16ecf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT14</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a41e750b67eb36c8da10328c565b90dd5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT15</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>ae40f5db1c57c98c6db42f15e0a56f03a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT16</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a7c55b9d74a6a1b129397792053cf08d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT17</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a27fe52b845a36280f50414ab4a00f74f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT18</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a56a026d146963b7d977255d9b1f682ae</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT19</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a9ce58ae33c478370e59c915b04b05381</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT20</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a83aaba3456aa46dfefe199fe6264d8dc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT21</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a1a5b4d4ca137f11bcb2e9c381f2ea6c5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT22</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>afdaa01ee37bdcd01ea44dbab6a30fd0d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT23</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>ada7be80971d1875e5c4774edd3ecd97d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT24</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a96cfb019bda32752ff4c8b8244aa6ea0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT25</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a3017291241a7269c1582154a3d3b1f09</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT26</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a4f97a8963cc15a1a50521d855b8a1331</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT27</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>afdd1584eaddf508717554b35a600b0fd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT28</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a4361544977e96fb8eb8387ff0feaf6b6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT29</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>adb368e517e545da53d8aace5923649e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT30</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>a70105c4a8c864754c8ba9e9b0e5da52a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>BIT31</name>
      <anchorfile>common_8h.html</anchorfile>
      <anchor>adfb09898dca36071e32cb1fbeec479e5</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>cortex.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>cortex_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>__CM_SAVER</name>
      <anchorfile>group__CM3__cortex__atomic__defines.html</anchorfile>
      <anchor>ga03ee33aa307c41ceb35288a40539ecbc</anchor>
      <arglist>(state)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CM_ATOMIC_BLOCK</name>
      <anchorfile>group__CM3__cortex__atomic__defines.html</anchorfile>
      <anchor>ga6904952be4c789ef4957c8c9bce83fc6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CM_ATOMIC_CONTEXT</name>
      <anchorfile>group__CM3__cortex__atomic__defines.html</anchorfile>
      <anchor>gabd996a426e72e63e0d4d100d3dd6ff87</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>cm_enable_interrupts</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga06a8f1e988ceacd262f4fbd14633b481</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>cm_disable_interrupts</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga30836716e88c3eccaf6c5fb872493450</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>cm_enable_faults</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga257d0ea270a0854aecb4856ab70f9c14</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>cm_disable_faults</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga0c65e90023b9244419cdb60b209c78f3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>cm_is_masked_interrupts</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>gacf65bc8e51a84d537167765189984f91</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>cm_is_masked_faults</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>gab3105858d73c77436822df3e9f4a0de6</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>cm_mask_interrupts</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga2cd22ed39720f21f182064caaa9ee05a</anchor>
      <arglist>(bool mask)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>cm_mask_faults</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga2716cd387030655888a4b35b766e0800</anchor>
      <arglist>(bool mask)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>__cm_atomic_set</name>
      <anchorfile>group__CM3__cortex__atomic__defines.html</anchorfile>
      <anchor>ga3250af4be4a3a81baae03814d898c507</anchor>
      <arglist>(bool *val)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>doc-cm3.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>doc-cm3_8h</filename>
  </compound>
  <compound kind="file">
    <name>dwt.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>dwt_8h</filename>
    <includes id="common_8h" name="common.h" local="no" imported="no">libopencm3/cm3/common.h</includes>
    <includes id="memorymap_8h" name="memorymap.h" local="no" imported="no">libopencm3/cm3/memorymap.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>DWT_CTRL</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>a90b9ebedff8635727698afd2fa84b90a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DWT_PCSR</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>a0c50769b8e0069fe1c5e06d1d356fad3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DWT_COMP</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>a38b233f00e629ebb65edd12a43bc42e9</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DWT_MASK</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>ab6fade424c853437a459b2d674303752</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DWT_FUNCTION</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>a0c1c91df0044ce1c9fb1b77c2cdca89a</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DWT_CTRL_NUMCOMP_SHIFT</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>a3f693d7a5c8e5eccc17cafb9d11b93f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DWT_CTRL_NUMCOMP</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>a816e74cbbafc9e19ed6b53bde399d656</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DWT_MASKx_MASK</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>adfa74f07ecb19bcc20f3c91eebb56535</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DWT_FUNCTIONx_MATCHED</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>a8270b3dbfb66d930c9bb8d561278d6c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DWT_FUNCTIONx_FUNCTION</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>ab5c4dd81d6986cbf20118e9663383cd8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>DWT_FUNCTIONx_FUNCTION_DISABLED</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>a482b48c83515ed4fedceac278036c764</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>dwt_enable_cycle_counter</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>a493f84a3f4912c8c1dd8c1371fbbc8e4</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>dwt_read_cycle_counter</name>
      <anchorfile>dwt_8h.html</anchorfile>
      <anchor>a4493afd58c9d262aa22776de5b51b6ee</anchor>
      <arglist>(void)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>fpb.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>fpb_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>FPB_CTRL</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>a162767eeac69e440b34262464fce181f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_REMAP</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>a2825f195122e711b4f758a6d40ae30e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_COMP</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>a57788ce6d93262ea87fcf266ce04cfe2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_LSR</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>a2f5f54cd11274358422ad95ce95fd510</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_LAR</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>a8f1e06736e694a4a4e4929c58289595b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_CTRL_NUM_CODE2_MASK</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>affc13bdab603d085c0f7bcd58350f406</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_CTRL_NUM_LIT_MASK</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>aa621fd596a273155a561d3bda7744f6c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_CTRL_NUM_CODE1_MASK</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>a7871d5bc782c4cb2104daf142e316461</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_CTRL_KEY</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>abea33e85db3bcfb7d22bbcf38f56c9e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_CTRL_ENABLE</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>a6cb89ee74732e34fcfd15985e7f35b9c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_COMP_REPLACE_REMAP</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>a28d1c763bdb5c697af1b9b0acc05bdff</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_COMP_REPLACE_BREAK_LOWER</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>ac5e1577afc727f19551f6d4f483436ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_COMP_REPLACE_BREAK_UPPER</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>a0c1dc5a1b9c977866f38aa89fad05862</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_COMP_REPLACE_BREAK_BOTH</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>ae02e83129123e467c43d9d1b866ff59f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_COMP_REPLACE_MASK</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>afe084d8a9e82d415ff7785e3105ba91f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>FPB_COMP_ENABLE</name>
      <anchorfile>fpb_8h.html</anchorfile>
      <anchor>a1a7409076397c752204cca3774cd9322</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>itm.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>itm_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>ITM_STIM8</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>abd4042d94a1484166f88bb42ec86b44f</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_STIM16</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>ae0edd6a4d2a32e569800c1a41b4f49c4</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_STIM32</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>afd1e8011912b712f1a88fe1a78f24d13</anchor>
      <arglist>(n)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TER</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>a18c94f3236b7d64c83e63cdd586e5e9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TPR</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>a4a106555b6d58115c86e09d48e199572</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>a935f0be267215442b7cffb0a191c7223</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_LSR</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>af69b121ad9855a63ecdca38a74895b44</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_LAR</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>af40951af1991b1b46200b122399187b3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_STIM_FIFOREADY</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>a2c7572d391a2bff1f5a92994bb004bcb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_BUSY</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>adc090027e495b730cfafe3198b091895</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_TRACE_BUS_ID_MASK</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>aa29bcf588bffc6673e1edd1bad317b6e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_TSPRESCALE_NONE</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>a2d3e743c011316d737160271f8838bbf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_TSPRESCALE_DIV4</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>a9087253adc655eb0f09e509e266823c0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_TSPRESCALE_DIV16</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>aa6494f215ba1154ec89cc7351f9d228d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_TSPRESCALE_DIV64</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>a9851904d56cddb843782654de575989a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_TSPRESCALE_MASK</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>ab7a22b7663d50a1345b7756fca53e0a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_SWOENA</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>af73f10e4b25f84d96d81c984da260561</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_TXENA</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>aa50e7ce101e78a99931a5cdf49a930bf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_SYNCENA</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>a06377f11d24d725f27fc4fe44b064c15</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_TSENA</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>a60c10f360235e2790cfc3fa174e596d7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>ITM_TCR_ITMENA</name>
      <anchorfile>itm_8h.html</anchorfile>
      <anchor>a3669715b5b18b1144dd0f69eb220e743</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>license.dox</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/</path>
    <filename>license_8dox</filename>
  </compound>
  <compound kind="file">
    <name>memorymap.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>memorymap_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>PPBI_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a19364e23013210d7c3939b513b514799</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>a3c14ed93192c8d9143322bbf77ebf770</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SYS_TICK_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>aa71477b62b12d865c7ef62cb768f7b2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>aa0288691785a5f868238e0468b39523d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_BASE</name>
      <anchorfile>memorymap_8h.html</anchorfile>
      <anchor>ad55a7ddb8d4b2398b0c1cfec76c0d9fd</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>mpu.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>mpu_8h</filename>
  </compound>
  <compound kind="file">
    <name>nvic.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>nvic_8h</filename>
    <includes id="common_8h" name="common.h" local="no" imported="no">libopencm3/cm3/common.h</includes>
    <includes id="memorymap_8h" name="memorymap.h" local="no" imported="no">libopencm3/cm3/memorymap.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_ISER</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga56d60698b9aa6c8bc051d3c81e1a1ee3</anchor>
      <arglist>(iser_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_ICER</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga3fa79c5ca67b7a9037cf9ddc28e43c00</anchor>
      <arglist>(icer_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_ISPR</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga6be3b05499bafab037e23f25d63d621c</anchor>
      <arglist>(ispr_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_ICPR</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gabbdb5811b29c0b99ebd769b35fc6b77d</anchor>
      <arglist>(icpr_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_IPR</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gac70cd532c336bcab3735403a1e0a8c48</anchor>
      <arglist>(ipr_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_NMI_IRQ</name>
      <anchorfile>group__nvic__sysint.html</anchorfile>
      <anchor>gabdeb6fa799341f6cf23d242d2e8e704b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_HARD_FAULT_IRQ</name>
      <anchorfile>group__nvic__sysint.html</anchorfile>
      <anchor>gaedbe55d3baf2439fb49301f0b5f9bc53</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_SV_CALL_IRQ</name>
      <anchorfile>group__nvic__sysint.html</anchorfile>
      <anchor>gaaefda4fe69bd9acbd73c12ff2d023ad2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_PENDSV_IRQ</name>
      <anchorfile>group__nvic__sysint.html</anchorfile>
      <anchor>ga60500110edbdb7d0992e8aef8fe04b7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_SYSTICK_IRQ</name>
      <anchorfile>group__nvic__sysint.html</anchorfile>
      <anchor>gafd650ffcb7f54e1f619d2ff3b1331d1f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>WEAK</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gad1480e9557edcc543498ca259cee6c7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>nvic_enable_irq</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gae5fdef5fd0dc9db35df8e84715fe8179</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>nvic_disable_irq</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gaf95cc3827a6e48d82c6046c639c80dc9</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>nvic_get_pending_irq</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga0af0d73b09caec78a330d202829391bf</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>nvic_set_pending_irq</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga3de46ef1bb9421e41fad4f407d0c8242</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>nvic_clear_pending_irq</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga55916a6ef4b3380692dc46bb0135386e</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>nvic_get_irq_enabled</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gaa2244a9f5f8c94a82bc8cf41d6ef254d</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>nvic_set_priority</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga404123c81365250fe09e0545b4c6bf66</anchor>
      <arglist>(uint8_t irqn, uint8_t priority)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>reset_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gaa13f582a41a6b190667d0ecd9c31072f</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>nmi_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga181d6892388ef1987ba92d0ae80e9359</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>hard_fault_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gae29640317be4df735067ff164aeeee0f</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>sv_call_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga38e39ab596274b8e97de76609cefa80a</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>pend_sv_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga214eb2c5d993ec1fe7b0a25dd428d3ae</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>sys_tick_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gae71d437809f2932d8f4e2e993732497d</anchor>
      <arglist>(void)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>scb.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>scb_8h</filename>
    <includes id="memorymap_8h" name="memorymap.h" local="no" imported="no">libopencm3/cm3/memorymap.h</includes>
    <includes id="common_8h" name="common.h" local="no" imported="no">libopencm3/cm3/common.h</includes>
    <class kind="struct">scb_exception_stack_frame</class>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ad6f841a4188701c78b4fbefc4dcd1cb6</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a7f8564488243827944de74c4db24b732</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_VTOR</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a4b62b78bccb6fe6afabe8f4969f58908</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_AIRCR</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>afb55c4d149b907fe569a1d8bb31cade1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SCR</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a046351cf4159d1e8953730326c31ec5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CCR</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a899566f16668d16cc5f7c7a1e9025bdd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>aee9878a9669b283c7a8b6cba2d32c2cc</anchor>
      <arglist>(shpr_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR1</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>af09aa434b2e83324ecf55cc2688ee163</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR2</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a8ae524551cd0a9cee2b7e4484d926c5a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR3</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ac0a168427d861c6175c07a2da89bca40</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_IMPLEMENTER_LSB</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a47c12d59106f05ad49c0b14ca2d313c7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_IMPLEMENTER</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a07d13461f7ac56baf2bc2005f49b08c9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_VARIANT_LSB</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ab5f00dc50a8de58ec563d87300957773</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_VARIANT</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a2918ac8b94d21ece6e60d8e57466b3ac</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_CONSTANT_LSB</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a69928ea936e108d9a57d5a1604578a76</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_CONSTANT</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a8590aaf8454c63e4cae33caea07e0b01</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_CONSTANT_ARMV6</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a62373802bfe1b94ea8acbc282be5b1dd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_CONSTANT_ARMV7</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a8c2a2c7833103aad36c175506c24fba5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_PARTNO_LSB</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a0f8e02abde5c1eddf2a08b45890d4b74</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_PARTNO</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a550badbbe87c076419c0cc1c914b6d3c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_REVISION_LSB</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ae78b5b55733d4de5e3c0775a0696926f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CPUID_REVISION</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a8d41122756e2a2a01f07f5863312a0b3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_NMIPENDSET</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a0a7d69b63652f05f4ff9b72d110dec7a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_PENDSVSET</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a4d08b3c1bd96c4c12dddd25aea063e35</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_PENDSVCLR</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a84b3c1eebacbbc3d33ecf875e2e298a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_PENDSTSET</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a1208f2e1fba16f8ce1fd533f48228898</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_PENDSTCLR</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a739c687961a5555b6a3903b617461892</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_ISRPREEMPT</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a699279156aae0333110fe24a5e4e3d21</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_ISRPENDING</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>addc9f4da4f73fd9aaeee3a8c97dac8c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_VECTPENDING_LSB</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>abbb04e3b21a113317ee838c58cab4da7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_VECTPENDING</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a91ba96d4d975d2ad3cd43c091b1e65af</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_RETOBASE</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a6c7ecfbff1db058bf1237fbab61e98bc</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_VECTACTIVE_LSB</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ab13f820f06f8fb0eec5d3b03f8c7c3dd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_ICSR_VECTACTIVE</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>aa03823cedb24b4d4c95812f121a2f493</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_AIRCR_VECTKEYSTAT_LSB</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a74ce167fa84d1dc80c497279e981f560</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_AIRCR_VECTKEYSTAT</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a107d7169f55d4443a7f4550e421a8adf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_AIRCR_VECTKEY</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ae9c09346491834693c481c5d5a20886d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_AIRCR_ENDIANESS</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ade5876f1c12d6322a188b09efe77f69d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_AIRCR_SYSRESETREQ</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a86c65d10100e2fb5fdcf826b2573b5d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_AIRCR_VECTCLRACTIVE</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ae3d9b3c94c860a0b0b038285ca817fd3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SCR_SEVEONPEND</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a01af8b70a406fa303e76bb5f2f916f63</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SCR_SLEEPDEEP</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ac4f4f02bfc91aef800b88fa58329cb92</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SCR_SLEEPONEXIT</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>aef484612839a04567ebaeeb57ca0b015</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CCR_STKALIGN</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a8c71d4e534d7d822ce32c3dec82bebd9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_CCR_UNALIGN_TRP</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a6a075d1f9722f6972ed1a98305e24cf9</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_4_MEMMANAGE</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a5b42da0b50cd1cecceb3176c999b1428</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_5_BUSFAULT</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a76547c355413147a90603d37b2d32311</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_6_USAGEFAULT</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a9be368c585755cab159031cc0797fc1b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_7_RESERVED</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a8d3ef2c46c0e94cdef699e93f118f355</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_8_RESERVED</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a6eccc3e0adca4618b60c4c439ea502e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_9_RESERVED</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ab5fc82d2ef9c4e3d3d02fc4c1fc355d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_10_RESERVED</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a2bc0e4c30aa8b3e84cb77d2038ebac14</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_11_SVCALL</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ac7afbca520df24fc9e934e4d79510f5b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_12_RESERVED</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>ab0cc15005e4b7f9ff250e5aab27cdfb0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_13_RESERVED</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a3eed56427e54cd9bd0a2896877989eef</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_14_PENDSV</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>aab3968811d19414caa0904053a1da39e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_SHPR_PRI_15_SYSTICK</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a84b60a3a05c2a04cc2118a866d13279e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCB_GET_EXCEPTION_STACK_FRAME</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>a0cc5fd2659edd23ae8abb4f4be7263ac</anchor>
      <arglist>(f)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>scb_reset_system</name>
      <anchorfile>scb_8h.html</anchorfile>
      <anchor>aad4d08be4e2b209f97d3276b7c7731e6</anchor>
      <arglist>(void)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>scs.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>scs_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SHPR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a0b01cbe730330dfdd47c4a779f804212</anchor>
      <arglist>(ipr_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>ad9ef5c6cd565da7e0e88397e4ca683ee</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DCRSR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>ac47b98497dac8466bda7b72d700c21f1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DCRDR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a9b626cc8621d525d1d494b62ad5230bf</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a558b8e7f0f8a8d4c2e421a7a44c9c662</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_DBGKEY</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a2a087621292aa48b1649e0a3e201647b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_C_DEBUGEN</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a67c8fb7221a677d42ffcab42d8553ee4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_C_HALT</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a83d13fee7b36c3bef763d0fa836d23cb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_C_STEP</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>ad063965f1a5b68a3454f978912ae4b9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_C_MASKINTS</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a7cdb38ce122b3397dec5fa39038e868d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_C_SNAPSTALL</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a1b68f42db3f1b18972e4a081d6f6e6a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_S_REGRDY</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>add8b7454b446e1dd705ddd6bfbd23216</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_S_HALT</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a21b476c02fbf5303f24e381b19fe8561</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_S_SLEEP</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a4ef92af060ec1fc6f2d6f3c730300171</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_S_LOCKUP</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a3f0ee88e7c62c32d6e01c32b56452953</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_S_RETIRE_ST</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a782b0e6536158702884e1de3aa1d7347</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DHCSR_S_RESET_ST</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a6e32bd6c73772ef68d60c96af6cb7f81</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DCRSR_REGSEL_MASK</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a10b8ba4f87e488d691d05eece0c87301</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DCRSR_REGSEL_XPSR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a13c0bff2d17e7fd1c378edbe414cc113</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DCRSR_REGSEL_MSP</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>ab4173a89891f145f94f2af55f3f4b27d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DCRSR_REGSEL_PSP</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a779da8ffbf9416747e0c3c51c55e4733</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_TRCENA</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>afdadd5f1ddd4f2c7dfd77c850d39e5f1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_MON_REQ</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a96effe83b4129231374c5133767ae6b0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_MON_STEP</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a528f32a82c1e89649febb0f24f35bf8e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_VC_MON_PEND</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>ac596479b9d7d77e35eeaa815c141b26f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_VC_MON_EN</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a12e6700779d8b3eeea475e7182a5d2db</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_VC_HARDERR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a773101ff25ee0c03eb0c17a6bc2099f5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_VC_INTERR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a8472cbcbb918035963cd959be26ba74c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_VC_BUSERR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a051e1c6a2d7896ee7d1a63e70fb86f89</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_VC_STATERR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a24c74f6c14add79e6aed09d20b6d8464</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_VC_CHKERR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a814d1ca673c011edc3d0b8a28cc8e259</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_VC_NOCPERR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>abf0f526e9cf772a548a1c5de956c8b1c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_VC_MMERR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>abd8a1631f8d700ec5ce7c157eb8aae6f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DEMCR_VC_CORERESET</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>af25da776d8974dc3538169a7dce8899b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYST_CSR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>ac86fa68498fbea9d8c1050b7eccb1901</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CM_SCS_SYST_RVR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a4b76425dbd3faeccacf2c1aa272900ba</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CM_SCS_SYST_CVR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a33a02d850c6ba4338215f2152dba99a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CM_SCS_SYST_CALIB</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a7c647467049955dbb723c077d74d3173</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYST_CSR_ENABLE</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a9016f6d43a42f02119b71c916ee479d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYST_CSR_TICKINT</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a4150fdb190e92163343b8a70853ed289</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYST_CSR_CLKSOURCE</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a446ca7f871610d2035028ebd3678216f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYST_CSR_COUNTFLAG</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a9d217cc683150a352e430d3bc745835e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYST_SYST_CALIB_TENMS_MASK</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a4638862b072e7cf5bc7571c8e3d2b18a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYST_SYST_CALIB_VALUE_INEXACT</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>ac158911b734266e60d8a219c41ae2747</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYST_SYST_CALIB_REF_NOT_IMPLEMENTED</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a0030bb054f0013f39ef1fa5c015a64a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_CTRL</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a087036b2f54569f4b4dc6d42b7357765</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_CYCCNT</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a7ffe017b37b33a6c0076cce4601d3fd0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_CPICNT</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a054eaa47bb48507d52f6bd673504c70f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_EXCCNT</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a387bb45a990aa3ff42a4e2e36c2960ed</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_SLEEPCNT</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>ac72a4893e2e3d9d65f5db8e3dffe5410</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_LSUCNT</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a90eb7f0698b261420a068bea54f6dc68</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_FOLDCNT</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>aaa205ae16ca604ddbe3ff463c3d39dfd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_PCSR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>ad170e87e8b9683fa51ea1708710214b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_LSR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a20662a57eb5361e053db9dc019eef7c0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_LAR</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>aa77e0ad0914bb7429d83b76306065e78</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_CTRL_CYCCNTENA</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a0814b32671ec81ef1b0fe1855200a343</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_LSR_SLK</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>aef62bdc10663a98d3edb9b1aa95937e3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_LSR_SLI</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a2c511dfedb698bc7fe5e98733e84e716</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_LAR_KEY</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a53433a110b199ad51ddbc4cc7a0eecd5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_DWT_CYCLE_COUNTER_ENABLED</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>ac9af4f7a9038682b67ea613d48d1cdd0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYSTICK_DISABLED</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>afa9bb946ea251db26e6935ff31ade593</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYSTICK_ENABLED</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a44368d8d72909fbb5de89f005463c971</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>SCS_SYSTICK_AND_IRQ_ENABLED</name>
      <anchorfile>scs_8h.html</anchorfile>
      <anchor>a603abd66d408869da6e74d20ac4c3159</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>sync.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>sync_8h</filename>
    <includes id="common_8h" name="common.h" local="yes" imported="no">common.h</includes>
    <member kind="function">
      <type>void</type>
      <name>__dmb</name>
      <anchorfile>sync_8h.html</anchorfile>
      <anchor>ad1ca1eff93298efc081ecc4b22ebed95</anchor>
      <arglist>(void)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>systick.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>systick_8h</filename>
    <includes id="memorymap_8h" name="memorymap.h" local="no" imported="no">libopencm3/cm3/memorymap.h</includes>
    <includes id="common_8h" name="common.h" local="no" imported="no">libopencm3/cm3/common.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga664a648a1ef9d90dd280176c5ec547c0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_RVR</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga6617c6912a90b37c034e0eeb2e12ec0b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CVR</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gafcba0d0cff7c416138b78b1ed1198b6e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CALIB</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga2008449261a84462413fc10e8e949ac4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_COUNTFLAG</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga0a6a69a143a4257ed517033163524886</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_CLKSOURCE_LSB</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga82177cdbc9fdc0126a2a74cf808d7969</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_CLKSOURCE</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga8f8123030d6e5d7d21a1ef3cbc815b5e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_CLKSOURCE_AHB_DIV8</name>
      <anchorfile>group__systick__clksource.html</anchorfile>
      <anchor>gadd7bddb11536d5eafa34bacbf5673811</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_CLKSOURCE_AHB</name>
      <anchorfile>group__systick__clksource.html</anchorfile>
      <anchor>gad4e608afdf933e1ac6656cd06681ade5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_TICKINT</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga829e6dbb8ecf50d36aa8258e2cf1587c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_ENABLE</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga803c330982edfbbeac7b14ab081e849d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_RVR_RELOAD</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga46f593f0f43644aeb67692011d4a5b14</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CVR_CURRENT</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga8cba67ffa9205688bd0478bd794c1a04</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CALIB_NOREF</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gab4de9cbeea2f3f747a9b02cbb6bf976a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CALIB_SKEW</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gab3b16fa14f4b2e1be896c24df74f92d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CALIB_TENMS</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga7a4687e38762c3838584db6d0ae136be</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_set_reload</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gaaa997323bf7f3378eb78a6277abc913f</anchor>
      <arglist>(uint32_t value)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>systick_set_frequency</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga7986a46de67e6396e366316f1c9e2437</anchor>
      <arglist>(uint32_t freq, uint32_t ahb)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>systick_get_reload</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gad26b5b9932a41cfe9e3600431b918831</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>systick_get_value</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gae6a4663107e824d7fe2936bdcbead94c</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_set_clocksource</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga13726e0964b31c85269beddbdc1d35e7</anchor>
      <arglist>(uint8_t clocksource)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_interrupt_enable</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga062163783d8d4106c8b945ed97b4decc</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_interrupt_disable</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gaee4e252973ddd7c5ffaa872da3b6a3fe</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_counter_enable</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gad1b556d07d6f0e3c0e55dc269e30a98a</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_counter_disable</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gac5d5225b9b6c96d9f1c3f2950717dcf4</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>systick_get_countflag</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga04dd39e8ef61f9d6be437a43fc16dcdf</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_clear</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga2604630453d0b6b35601375d0ee7e4a0</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>systick_get_calib</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga4351bcd52c07a23b5b999b062c123084</anchor>
      <arglist>(void)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>tpiu.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>tpiu_8h</filename>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_SSPSR</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>afe9850f6644e019acbd41ae36f2a9bb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_CSPSR</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a30ef8dde42bb8b5ce47fd60b3450b4ce</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_ACPR</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a5b928b2763dbc7685675954dfbc77b5e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_SPPR</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a07413fb90b689fafae5fb669e7bfadd7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_FFSR</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a020c57a36fab7ac79ff2310e34b8d708</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_FFCR</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>ac8a5a5f384ba44b32ca65e20ed86745b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_DEVID</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>ad9b75f6e89de1971b490ad69ffff4c0c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_LSR</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>af953912baf13340fd4a81cfb4ad71d99</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_LAR</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>ab976c60354252320159aac96b0b328bd</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_SPPR_SYNC</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a819c9d9fcf2a17f910382d0747342523</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_SPPR_ASYNC_MANCHESTER</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a3eba7b77a69dc4e4f41360c6b77dea72</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_SPPR_ASYNC_NRZ</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a93c1f6789d79fbfce3d9be9c207fdeb8</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_FFSR_FTNONSTOP</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a8d49e36884f30b37503ee137ba6555a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_FFSR_TCPRESENT</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a06f2d73ced3d6b19c9170bfdaa058b0c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_FFSR_FTSTOPPED</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a73e49a21a5eb0e0e16448cb1f083aecb</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_FFSR_FLINPROG</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>ab1fe105f0466d5161ead63d2d14a7085</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_FFCR_TRIGIN</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a72cb9a2c1ad3196f2fdfcc2eb71accb5</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPIU_FFCR_ENFCONT</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>ab7cac7937bdae1a69d7160da837e38ee</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPUI_DEVID_NRZ_SUPPORTED</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a35b5d2f1df1b7b2bf5aa26ca581f536f</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPUI_DEVID_MANCHESTER_SUPPORTED</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a3984ad73b9e15f6507ec771d6ea1a9ec</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>TPUI_DEVID_FIFO_SIZE_MASK</name>
      <anchorfile>tpiu_8h.html</anchorfile>
      <anchor>a3831444239089a4bcf01c5c9cc849008</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>vector.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/cm3/</path>
    <filename>vector_8h</filename>
    <includes id="common_8h" name="common.h" local="no" imported="no">libopencm3/cm3/common.h</includes>
    <includes id="nvic_8h" name="nvic.h" local="no" imported="no">libopencm3/cm3/nvic.h</includes>
    <class kind="struct">vector_table_t</class>
    <member kind="typedef">
      <type>void(*</type>
      <name>vector_table_entry_t</name>
      <anchorfile>vector_8h.html</anchorfile>
      <anchor>a643ad4ed6072ebd4eb5412218ee0e0e1</anchor>
      <arglist>)(void)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>scb_exception_stack_frame</name>
    <filename>structscb__exception__stack__frame.html</filename>
    <member kind="variable">
      <type>uint32_t</type>
      <name>r0</name>
      <anchorfile>structscb__exception__stack__frame.html</anchorfile>
      <anchor>ae7ebc5fa67cc65d5039f8f978e3a2062</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>r1</name>
      <anchorfile>structscb__exception__stack__frame.html</anchorfile>
      <anchor>a9fea68a34dc0a9d385390205092162e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>r2</name>
      <anchorfile>structscb__exception__stack__frame.html</anchorfile>
      <anchor>ac38c8c267ebaa36b9e8114e0c8da6b33</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>r3</name>
      <anchorfile>structscb__exception__stack__frame.html</anchorfile>
      <anchor>a974713f74276b9c27efaf8dd8e066784</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>r12</name>
      <anchorfile>structscb__exception__stack__frame.html</anchorfile>
      <anchor>a0e6126588086c406b720b3961bda7311</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>lr</name>
      <anchorfile>structscb__exception__stack__frame.html</anchorfile>
      <anchor>ac66306df89b575b3c3ae413f89211d8d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>pc</name>
      <anchorfile>structscb__exception__stack__frame.html</anchorfile>
      <anchor>a44ebd35215d8e75f0154f26cc14165bc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>xpsr</name>
      <anchorfile>structscb__exception__stack__frame.html</anchorfile>
      <anchor>a2e6e69032909f740145d93bac2fec49b</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>vector_table_t</name>
    <filename>structvector__table__t.html</filename>
    <member kind="variable">
      <type>unsigned int *</type>
      <name>initial_sp_value</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>aeb5ce1efa9b9ecd868829beabf24fd91</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>reset</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>adc4942c678a88cab8494a58f80428077</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>nmi</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>a7da79b1d8184e743b15a5c878250b68a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>hard_fault</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>a1489006508850f377a98cd297fa808bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>memory_manage_fault</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>a0f471d2e9ba10298c9a292dbf59aafde</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>bus_fault</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>a4c2b5debe7567df33665bca4ab8a4b2c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>usage_fault</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>a9bc2b639e5ac8019bfad3a97cb35e414</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>reserved_x001c</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>a6578b9135633f588aee27e73367546e9</anchor>
      <arglist>[4]</arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>sv_call</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>ace5b229bd0e04cbc018c8457168640a4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>debug_monitor</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>a7bd35c699bccae1299b1f7dbdc049106</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>reserved_x0034</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>aed397ae787125497ae9869cbe2673500</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>pend_sv</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>afe26c2d806f2caef1935e36c053d7023</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>systick</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>acf9d4ecb17d3ebecbcbd24966c18102f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>vector_table_entry_t</type>
      <name>irq</name>
      <anchorfile>structvector__table__t.html</anchorfile>
      <anchor>a14df56a3e6131a72bcd0298a0187239e</anchor>
      <arglist>[NVIC_IRQ_COUNT]</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>debugging</name>
    <title>Debugging</title>
    <filename>group__debugging.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>CM3_LIKELY</name>
      <anchorfile>group__debugging.html</anchorfile>
      <anchor>ga2b90b6f44540e6e706864903ddc35926</anchor>
      <arglist>(expr)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>cm3_assert</name>
      <anchorfile>group__debugging.html</anchorfile>
      <anchor>gaac76fc08d5852b8bdcf33b5e6448355e</anchor>
      <arglist>(expr)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>cm3_assert_not_reached</name>
      <anchorfile>group__debugging.html</anchorfile>
      <anchor>gac2ec555ba39f6c80aa9f3a9289864076</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cm3_assert_failed</name>
      <anchorfile>group__debugging.html</anchorfile>
      <anchor>ga9c0fa84fe9c0e99ad57a3d079ba41ddc</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>cm3_assert_failed_verbose</name>
      <anchorfile>group__debugging.html</anchorfile>
      <anchor>ga93d8d0dfa33f2bc46e3fa201813ef3bc</anchor>
      <arglist>(const char *file, int line, const char *func, const char *assert_expr)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>CM3_cortex_defines</name>
    <title>Cortex Core Defines</title>
    <filename>group__CM3__cortex__defines.html</filename>
    <subgroup>CM3_cortex_atomic_defines</subgroup>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>cm_enable_interrupts</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga06a8f1e988ceacd262f4fbd14633b481</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>cm_disable_interrupts</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga30836716e88c3eccaf6c5fb872493450</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>cm_enable_faults</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga257d0ea270a0854aecb4856ab70f9c14</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>cm_disable_faults</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga0c65e90023b9244419cdb60b209c78f3</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>cm_is_masked_interrupts</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>gacf65bc8e51a84d537167765189984f91</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>cm_is_masked_faults</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>gab3105858d73c77436822df3e9f4a0de6</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>cm_mask_interrupts</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga2cd22ed39720f21f182064caaa9ee05a</anchor>
      <arglist>(bool mask)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>cm_mask_faults</name>
      <anchorfile>group__CM3__cortex__defines.html</anchorfile>
      <anchor>ga2716cd387030655888a4b35b766e0800</anchor>
      <arglist>(bool mask)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>CM3_cortex_atomic_defines</name>
    <title>Cortex Core Atomic support Defines</title>
    <filename>group__CM3__cortex__atomic__defines.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>__CM_SAVER</name>
      <anchorfile>group__CM3__cortex__atomic__defines.html</anchorfile>
      <anchor>ga03ee33aa307c41ceb35288a40539ecbc</anchor>
      <arglist>(state)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CM_ATOMIC_BLOCK</name>
      <anchorfile>group__CM3__cortex__atomic__defines.html</anchorfile>
      <anchor>ga6904952be4c789ef4957c8c9bce83fc6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>CM_ATOMIC_CONTEXT</name>
      <anchorfile>group__CM3__cortex__atomic__defines.html</anchorfile>
      <anchor>gabd996a426e72e63e0d4d100d3dd6ff87</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static bool</type>
      <name>__cm_atomic_set</name>
      <anchorfile>group__CM3__cortex__atomic__defines.html</anchorfile>
      <anchor>ga3250af4be4a3a81baae03814d898c507</anchor>
      <arglist>(bool *val)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>CM3_defines</name>
    <title>CM3 Defines</title>
    <filename>group__CM3__defines.html</filename>
    <subgroup>CM3_cortex_defines</subgroup>
    <subgroup>CM3_nvic_defines</subgroup>
    <subgroup>CM3_systick_defines</subgroup>
  </compound>
  <compound kind="group">
    <name>CM3_nvic_defines</name>
    <title>NVIC Defines</title>
    <filename>group__CM3__nvic__defines.html</filename>
    <subgroup>nvic_sysint</subgroup>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_ISER</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga56d60698b9aa6c8bc051d3c81e1a1ee3</anchor>
      <arglist>(iser_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_ICER</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga3fa79c5ca67b7a9037cf9ddc28e43c00</anchor>
      <arglist>(icer_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_ISPR</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga6be3b05499bafab037e23f25d63d621c</anchor>
      <arglist>(ispr_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_ICPR</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gabbdb5811b29c0b99ebd769b35fc6b77d</anchor>
      <arglist>(icpr_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_IPR</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gac70cd532c336bcab3735403a1e0a8c48</anchor>
      <arglist>(ipr_id)</arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>WEAK</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gad1480e9557edcc543498ca259cee6c7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>nvic_enable_irq</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gae5fdef5fd0dc9db35df8e84715fe8179</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>nvic_disable_irq</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gaf95cc3827a6e48d82c6046c639c80dc9</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>nvic_get_pending_irq</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga0af0d73b09caec78a330d202829391bf</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>nvic_set_pending_irq</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga3de46ef1bb9421e41fad4f407d0c8242</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>nvic_clear_pending_irq</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga55916a6ef4b3380692dc46bb0135386e</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>nvic_get_irq_enabled</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gaa2244a9f5f8c94a82bc8cf41d6ef254d</anchor>
      <arglist>(uint8_t irqn)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>nvic_set_priority</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga404123c81365250fe09e0545b4c6bf66</anchor>
      <arglist>(uint8_t irqn, uint8_t priority)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>reset_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gaa13f582a41a6b190667d0ecd9c31072f</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>nmi_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga181d6892388ef1987ba92d0ae80e9359</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>hard_fault_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gae29640317be4df735067ff164aeeee0f</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>sv_call_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga38e39ab596274b8e97de76609cefa80a</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>pend_sv_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>ga214eb2c5d993ec1fe7b0a25dd428d3ae</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void WEAK</type>
      <name>sys_tick_handler</name>
      <anchorfile>group__CM3__nvic__defines.html</anchorfile>
      <anchor>gae71d437809f2932d8f4e2e993732497d</anchor>
      <arglist>(void)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>nvic_sysint</name>
    <title>Cortex M0/M3/M4 System Interrupts</title>
    <filename>group__nvic__sysint.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_NMI_IRQ</name>
      <anchorfile>group__nvic__sysint.html</anchorfile>
      <anchor>gabdeb6fa799341f6cf23d242d2e8e704b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_HARD_FAULT_IRQ</name>
      <anchorfile>group__nvic__sysint.html</anchorfile>
      <anchor>gaedbe55d3baf2439fb49301f0b5f9bc53</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_SV_CALL_IRQ</name>
      <anchorfile>group__nvic__sysint.html</anchorfile>
      <anchor>gaaefda4fe69bd9acbd73c12ff2d023ad2</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_PENDSV_IRQ</name>
      <anchorfile>group__nvic__sysint.html</anchorfile>
      <anchor>ga60500110edbdb7d0992e8aef8fe04b7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>NVIC_SYSTICK_IRQ</name>
      <anchorfile>group__nvic__sysint.html</anchorfile>
      <anchor>gafd650ffcb7f54e1f619d2ff3b1331d1f</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>CM3_systick_defines</name>
    <title>SysTick Defines</title>
    <filename>group__CM3__systick__defines.html</filename>
    <subgroup>systick_clksource</subgroup>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga664a648a1ef9d90dd280176c5ec547c0</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_RVR</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga6617c6912a90b37c034e0eeb2e12ec0b</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CVR</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gafcba0d0cff7c416138b78b1ed1198b6e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CALIB</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga2008449261a84462413fc10e8e949ac4</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_COUNTFLAG</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga0a6a69a143a4257ed517033163524886</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_CLKSOURCE_LSB</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga82177cdbc9fdc0126a2a74cf808d7969</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_CLKSOURCE</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga8f8123030d6e5d7d21a1ef3cbc815b5e</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_TICKINT</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga829e6dbb8ecf50d36aa8258e2cf1587c</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_ENABLE</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga803c330982edfbbeac7b14ab081e849d</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_RVR_RELOAD</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga46f593f0f43644aeb67692011d4a5b14</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CVR_CURRENT</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga8cba67ffa9205688bd0478bd794c1a04</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CALIB_NOREF</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gab4de9cbeea2f3f747a9b02cbb6bf976a</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CALIB_SKEW</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gab3b16fa14f4b2e1be896c24df74f92d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CALIB_TENMS</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga7a4687e38762c3838584db6d0ae136be</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_set_reload</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gaaa997323bf7f3378eb78a6277abc913f</anchor>
      <arglist>(uint32_t value)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>systick_set_frequency</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga7986a46de67e6396e366316f1c9e2437</anchor>
      <arglist>(uint32_t freq, uint32_t ahb)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>systick_get_reload</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gad26b5b9932a41cfe9e3600431b918831</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>systick_get_value</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gae6a4663107e824d7fe2936bdcbead94c</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_set_clocksource</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga13726e0964b31c85269beddbdc1d35e7</anchor>
      <arglist>(uint8_t clocksource)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_interrupt_enable</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga062163783d8d4106c8b945ed97b4decc</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_interrupt_disable</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gaee4e252973ddd7c5ffaa872da3b6a3fe</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_counter_enable</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gad1b556d07d6f0e3c0e55dc269e30a98a</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_counter_disable</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>gac5d5225b9b6c96d9f1c3f2950717dcf4</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>systick_get_countflag</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga04dd39e8ef61f9d6be437a43fc16dcdf</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>systick_clear</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga2604630453d0b6b35601375d0ee7e4a0</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function">
      <type>uint32_t</type>
      <name>systick_get_calib</name>
      <anchorfile>group__CM3__systick__defines.html</anchorfile>
      <anchor>ga4351bcd52c07a23b5b999b062c123084</anchor>
      <arglist>(void)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>systick_clksource</name>
    <title>Clock source selection</title>
    <filename>group__systick__clksource.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_CLKSOURCE_AHB_DIV8</name>
      <anchorfile>group__systick__clksource.html</anchorfile>
      <anchor>gadd7bddb11536d5eafa34bacbf5673811</anchor>
      <arglist></arglist>
    </member>
    <member kind="define">
      <type>#define</type>
      <name>STK_CSR_CLKSOURCE_AHB</name>
      <anchorfile>group__systick__clksource.html</anchorfile>
      <anchor>gad4e608afdf933e1ac6656cd06681ade5</anchor>
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
    <title>libopencm3 Core CM3</title>
    <filename>index</filename>
  </compound>
</tagfile>

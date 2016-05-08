<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>audio.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/usb/</path>
    <filename>audio_8h</filename>
    <class kind="struct">usb_audio_header_descriptor_head</class>
    <class kind="struct">usb_audio_header_descriptor_body</class>
    <member kind="function">
      <type>struct usb_audio_header_descriptor_head</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__audio__defines.html</anchorfile>
      <anchor>gaaee0a5ff0aa053fd97f61a97733cac25</anchor>
      <arglist>((packed))</arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>audio_8h.html</anchorfile>
      <anchor>a1deca1f1d6e5815b290e6e1015bce5b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>audio_8h.html</anchorfile>
      <anchor>a67d7027b9eb9ed268d28e84fbc675707</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>audio_8h.html</anchorfile>
      <anchor>ac9a7544f66e5343048a779cc2f81f79e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdADC</name>
      <anchorfile>audio_8h.html</anchorfile>
      <anchor>ada1b9320cf3664015c08cb88a94da803</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wTotalLength</name>
      <anchorfile>audio_8h.html</anchorfile>
      <anchor>a866c5d9bb8a60f25805d37b7a5b5f9e8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>binCollection</name>
      <anchorfile>audio_8h.html</anchorfile>
      <anchor>adb69d5d9a93742e74fc972022992561c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>baInterfaceNr</name>
      <anchorfile>audio_8h.html</anchorfile>
      <anchor>ae4fde58bc5cb5b731f95bc7e0a45695e</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>cdc.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/usb/</path>
    <filename>cdc_8h</filename>
    <class kind="struct">usb_cdc_header_descriptor</class>
    <class kind="struct">usb_cdc_union_descriptor</class>
    <class kind="struct">usb_cdc_call_management_descriptor</class>
    <class kind="struct">usb_cdc_acm_descriptor</class>
    <class kind="struct">usb_cdc_line_coding</class>
    <class kind="struct">usb_cdc_notification</class>
    <member kind="enumeration">
      <type></type>
      <name>usb_cdc_line_coding_bCharFormat</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>ga7edb29528e9dcced43aaca511dbd9bc2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_1_STOP_BITS</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga7edb29528e9dcced43aaca511dbd9bc2a20790d5ae5573b13e38a5848ed4f8437</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_1_5_STOP_BITS</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga7edb29528e9dcced43aaca511dbd9bc2a528bc870457b07f3e1df4f54b89aef77</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_2_STOP_BITS</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga7edb29528e9dcced43aaca511dbd9bc2a88105b55058eb7b4e1a22f0e442801b9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>usb_cdc_line_coding_bParityType</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>ga0cabe0e00dd7200ff5e90f0119fbf66f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_NO_PARITY</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga0cabe0e00dd7200ff5e90f0119fbf66fa84dcd5abce5bd2c699749b70511bf95a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_ODD_PARITY</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga0cabe0e00dd7200ff5e90f0119fbf66fa2c2ea42ec25ac66d613f57416fa4b35d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_EVEN_PARITY</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga0cabe0e00dd7200ff5e90f0119fbf66fa1a69931078ca04ed352f2a850f42ba63</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_MARK_PARITY</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga0cabe0e00dd7200ff5e90f0119fbf66fa3ce606e8526fd03fef527027bb28b160</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_SPACE_PARITY</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga0cabe0e00dd7200ff5e90f0119fbf66fa4ff36fa9da45d9cbcbc10638e23a32dc</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>struct usb_cdc_header_descriptor</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gab6677e82689abb017cf4e5590b1305fa</anchor>
      <arglist>((packed))</arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionLength</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a65d7922f01732e3c44ebb46a73f99393</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a67d7027b9eb9ed268d28e84fbc675707</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>ac9a7544f66e5343048a779cc2f81f79e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdCDC</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a2bffb583550e4d83b4a2a4d735ece97f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bControlInterface</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>af8b3596257cbd25892a39fbb63b9862b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bSubordinateInterface0</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a99c623f394921bb203f6fd21b124a600</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmCapabilities</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a881b7a2d441c1074050bb06c058b680c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDataInterface</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a69219cd97dd6c69e7d5b798d9251ce5f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dwDTERate</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a1c7eaa2f09ca8253de1711d01e05e02c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bCharFormat</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a296b7b4f7eaba228afa94b5f9854b86b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bParityType</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a47f5ab7fcc915dedd560214e61438bc3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDataBits</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>aeb01baccbd5328de664ba39a4c295941</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>enum usb_cdc_line_coding_bCharFormat</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>ga8cb92f50fd0c1ad2ab1195289f72bff6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmRequestType</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>abea120ea756eb0abdb8aafb8713f61f4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNotification</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>aad103f9662f9e0805f460ca3ce27c5bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wValue</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a7383d0165ad8eab48aa239c005199121</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wIndex</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>adc578ce35d89d52deba61ee6f312f177</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wLength</name>
      <anchorfile>cdc_8h.html</anchorfile>
      <anchor>a496c03443b177fd2e6c93616064d2934</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>dfu.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/usb/</path>
    <filename>dfu_8h</filename>
    <class kind="struct">usb_dfu_descriptor</class>
    <member kind="enumeration">
      <type></type>
      <name>dfu_req</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ga1eb1afd96bccc953e4c19022df7d2171</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_DETACH</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171ab8e18a7fd7b5b1899c5b57fa69faac69</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_DNLOAD</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171aa3d14f787e4880e7c6b931f05070330f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_UPLOAD</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171af7427780a3977ec43a36b102c639289c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_GETSTATUS</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171a29e1b58118f9b9e096b293b365bdc91b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_CLRSTATUS</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171a8adc62cb245219266a0a567053633a1e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_GETSTATE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171a07c028e3b420dac6bf986ffbe971d2f0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_ABORT</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171ac4cdec2bd3bc05311e89fc989ca0c61d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>dfu_status</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gabcf2757cf1c6281a06a8a6f25ff2aa31</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_OK</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a3a550674419538042b365d365595dc40</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_TARGET</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a9a7a7639b542fa571c5bf230e811afa5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_FILE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a507bb0dd505ecf8e1ad31f3ad59ca595</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_WRITE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a714bc30afe28582ddb38e90702beb1d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_ERASE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a69f5911f3539cf83984e2220e069d9d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_CHECK_ERASED</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a1b79f547824d24336bd32c4facf90e35</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_PROG</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a334d3e00203a36c177a36c4f20ca4cd3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_VERIFY</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31ab209da0e18d2bdac90cce75446f5e284</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_ADDRESS</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31ab7abfa9c48bb721cdbc63a220e0a34a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_NOTDONE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a9f0b8e1bdb57da25a84e499db614fa38</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_FIRMWARE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31aaebd82c354d409a6a918f103bbfe6c26</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_VENDOR</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31ad5c270162ee2af751292540632a538c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_USBR</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31abc69336005f3df189e7df6d0f0ad69b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_POR</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a4b8d062347f43c1d75a58ed4e74d63bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_UNKNOWN</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31afeee9594f62632238db9053c37a99465</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_STALLEDPKT</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a8d963dfec7ec03c538c5a38944f63fe3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>dfu_state</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ga3f025bde53c777c5e0493ef54bbeb143</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_APP_IDLE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a2071f71d4f5fb2435f161d1a70167ec1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_APP_DETACH</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a1cae6396e56ba1edfd3bc7bf4b9b6dd4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_IDLE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a72332acd446920c416a5b4da079b33d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_DNLOAD_SYNC</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a2c74cfc90fa6f9e9df8bce3fca9d3ed4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_DNBUSY</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a262a372a18004a540f3486c9d6eb77de</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_DNLOAD_IDLE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a5658b351cf06fcd7af8f11f911444dab</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_MANIFEST_SYNC</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a779788d8fec7d6397b6a352ce62940b7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_MANIFEST</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143acd82927685281eb302cf3b33330cb4ac</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_MANIFEST_WAIT_RESET</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a15938a3a729a93ff2003e7c247be8371</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_UPLOAD_IDLE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a86181ad59e744acb655eaa940cdf42d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_ERROR</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143ac2c6d30505ea22bcc31222a41cb4815b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>struct usb_dfu_descriptor</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ga53297a7dd343d62a8a8efde4909b3248</anchor>
      <arglist>((packed))</arglist>
    </member>
    <member kind="variable">
      <type>enum dfu_req</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gafab3ce003dc4e9015c96f17748178a0f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>dfu_8h.html</anchorfile>
      <anchor>a1deca1f1d6e5815b290e6e1015bce5b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>dfu_8h.html</anchorfile>
      <anchor>a67d7027b9eb9ed268d28e84fbc675707</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmAttributes</name>
      <anchorfile>dfu_8h.html</anchorfile>
      <anchor>ae7c63b4cda39953db5710012a5d91c6f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wDetachTimeout</name>
      <anchorfile>dfu_8h.html</anchorfile>
      <anchor>aba725b47e591dfa9a30a01e80fa1aa49</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wTransferSize</name>
      <anchorfile>dfu_8h.html</anchorfile>
      <anchor>abf7c6492df9b515329616580afb383d2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdDFUVersion</name>
      <anchorfile>dfu_8h.html</anchorfile>
      <anchor>a7e5857b7c9ed4541e1c7283c2d3f44af</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>doc-usb.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/usb/</path>
    <filename>doc-usb_8h</filename>
  </compound>
  <compound kind="file">
    <name>hid.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/usb/</path>
    <filename>hid_8h</filename>
    <class kind="struct">usb_hid_descriptor</class>
    <member kind="function">
      <type>struct usb_hid_descriptor</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__hid__defines.html</anchorfile>
      <anchor>gab06170e95e244e47f1d5c11ced1a483f</anchor>
      <arglist>((packed))</arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>hid_8h.html</anchorfile>
      <anchor>a1deca1f1d6e5815b290e6e1015bce5b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>hid_8h.html</anchorfile>
      <anchor>a67d7027b9eb9ed268d28e84fbc675707</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdHID</name>
      <anchorfile>hid_8h.html</anchorfile>
      <anchor>acf09c753851ce018b81a07325bda9d8e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bCountryCode</name>
      <anchorfile>hid_8h.html</anchorfile>
      <anchor>a969a604033b72573d59a740badf6920e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumDescriptors</name>
      <anchorfile>hid_8h.html</anchorfile>
      <anchor>a86ff434026085d9c0c751185eb3700c5</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>license.dox</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/</path>
    <filename>license_8dox</filename>
  </compound>
  <compound kind="file">
    <name>midi.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/usb/</path>
    <filename>midi_8h</filename>
    <class kind="struct">usb_midi_header_descriptor</class>
    <class kind="struct">usb_midi_in_jack_descriptor</class>
    <class kind="struct">usb_midi_out_jack_descriptor_head</class>
    <class kind="struct">usb_midi_out_jack_descriptor_body</class>
    <class kind="struct">usb_midi_out_jack_descriptor_tail</class>
    <class kind="struct">usb_midi_out_jack_descriptor</class>
    <class kind="struct">usb_midi_element_descriptor_head</class>
    <class kind="struct">usb_midi_element_descriptor_body</class>
    <class kind="struct">usb_midi_element_descriptor_tail</class>
    <class kind="struct">usb_midi_element_descriptor</class>
    <class kind="struct">usb_midi_endpoint_descriptor_head</class>
    <class kind="struct">usb_midi_endpoint_descriptor_body</class>
    <class kind="struct">usb_midi_endpoint_descriptor</class>
    <member kind="function">
      <type>struct usb_audio_header_descriptor_head</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__audio__defines.html</anchorfile>
      <anchor>gaaee0a5ff0aa053fd97f61a97733cac25</anchor>
      <arglist>((packed))</arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a1deca1f1d6e5815b290e6e1015bce5b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a67d7027b9eb9ed268d28e84fbc675707</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>ac9a7544f66e5343048a779cc2f81f79e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdMSC</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>af4b12e83986d7c36f6f0b8a3a91165de</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wTotalLength</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a866c5d9bb8a60f25805d37b7a5b5f9e8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bJackType</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a3b5975afbb0b59086a87f230def8b686</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bJackID</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a2a87d2bf8fba9c5650a0c93a5fd9dd28</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iJack</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>af18ec08a72080e93d4b2ece547365ff3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNrInputPins</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a01ba090448636e4decff2485918c890e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>baSourceID</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>af05897f677536d4f86aa4e48f2655e2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>baSourcePin</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a16ce648a3a7989830bbe4570588e6a59</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct usb_midi_out_jack_descriptor_head</type>
      <name>head</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>aba8e093cf063a0c36956058434669139</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct usb_midi_out_jack_descriptor_body</type>
      <name>source</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a2c1444226bf3b33e9bb11e4f68b60cd0</anchor>
      <arglist>[1]</arglist>
    </member>
    <member kind="variable">
      <type>struct usb_midi_out_jack_descriptor_tail</type>
      <name>tail</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>af066436a27cb9606c5ab4249aa3f3055</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bElementID</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a8dbdc392175a725e3dedbd67fe47f45a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNrOutputPins</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a17e60856c03aff1343beb51f5262be52</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInTerminalLink</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a4dc207e44112476206d878e490b21974</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bOutTerminalLink</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a2232352d0a0dab78271ab003b7b69ee6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bElCapsSize</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a624f0db8374fd0c4c5d49261c7efce95</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bmElementCaps</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a5406a096e780410e35305c4dbe3e848f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iElement</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>aa2ff4f5935dab24f9e9a8bd13de7956a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubType</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a7569c561e3556e7df6b24aed1ff66f28</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumEmbMIDIJack</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a843bf89cebf5ee2f1372fe3a22f36dab</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>baAssocJackID</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>af5d629eaa4cb5907a3657e1ee3d8112d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct usb_midi_endpoint_descriptor_body</type>
      <name>jack</name>
      <anchorfile>midi_8h.html</anchorfile>
      <anchor>a135f5f40eefc4e98097e8464126b85e8</anchor>
      <arglist>[1]</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>msc.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/usb/</path>
    <filename>msc_8h</filename>
    <member kind="typedef">
      <type>struct _usbd_mass_storage</type>
      <name>usbd_mass_storage</name>
      <anchorfile>group__usb__msc__defines.html</anchorfile>
      <anchor>ga38f5f9afaaae0a35c540c5d2d59dfd57</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>usbd_mass_storage *</type>
      <name>usb_msc_init</name>
      <anchorfile>group__usb__msc__defines.html</anchorfile>
      <anchor>ga5e6959c3ac6ff4efab4fd3b59353f497</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ep_in, uint8_t ep_in_size, uint8_t ep_out, uint8_t ep_out_size, const char *vendor_id, const char *product_id, const char *product_revision_level, const uint32_t block_count, int(*read_block)(uint32_t lba, uint8_t *copy_to), int(*write_block)(uint32_t lba, const uint8_t *copy_from))</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/usb/</path>
    <filename>usb_8c</filename>
    <member kind="function">
      <type>usbd_device *</type>
      <name>usbd_init</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga8bbc488106ca72c2b550c1c9f89d5748</anchor>
      <arglist>(const usbd_driver *driver, const struct usb_device_descriptor *dev, const struct usb_config_descriptor *conf, const char **strings, int num_strings, uint8_t *control_buffer, uint16_t control_buffer_size)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_reset_callback</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga2cdd6f2cd24e7e222b2add6e08283a7e</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_suspend_callback</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga064f9d058427b1fb6f40b35e03a8cd5b</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_resume_callback</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga004c77a73e4dcd6216678925f3fa1f41</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_sof_callback</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>gad8e688f14cac90c4933954e3ebbfc3b7</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>_usbd_reset</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga3bc8dd33500621a1e900c78a42dcaf27</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_poll</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga94eba5aff7befd8009cea2f2abb43134</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_disconnect</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>gaef20aadd330e4bb9e97635eeb751d8f9</anchor>
      <arglist>(usbd_device *usbd_dev, bool disconnected)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_setup</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga888110ac1598406bb834b494c8db7f6c</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t type, uint16_t max_size, void(*callback)(usbd_device *usbd_dev, uint8_t ep))</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>usbd_ep_write_packet</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>gaa5b245bdd9757cdc79d1fe721ebcf03f</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, const void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>usbd_ep_read_packet</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>gabd42dc86361dbf9c639fd883ada1f572</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_stall_set</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga6d16be856512df298e40dfa31a216172</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t stall)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>usbd_ep_stall_get</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga396b34ccbe5bf6718417ef3ee0c3b788</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_nak_set</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga2f2a7dac5031fd8d1b5eb076d3f24b42</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t nak)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb_control.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/usb/</path>
    <filename>usb__control_8c</filename>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>stall_transaction</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>gaf6f522923d66ffdea79fbbd4decc31d4</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>usbd_register_control_callback</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>ga86f80b7593ee05d364fd791896213e38</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t type, uint8_t type_mask, usbd_control_callback callback)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>usb_control_send_chunk</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>ga46375610ee21a2c8ab4a06fe3bd70291</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_control_recv_chunk</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>gaf3b08c6a689c9f5a20f636507e70bb52</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_control_request_dispatch</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>ga883123627c0bed716ec41d4d7f3c0a46</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>usb_control_setup_read</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>ga2149cd30a26a18c9c46e32db4aa5ef3e</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>usb_control_setup_write</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>gab9305438ecc3aaa7359aacdbc4f55c29</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>_usbd_control_setup</name>
      <anchorfile>usb__control_8c.html</anchorfile>
      <anchor>ad4bc495b7e757434644e1dece336b261</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ea)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>_usbd_control_out</name>
      <anchorfile>usb__control_8c.html</anchorfile>
      <anchor>a1a921e8785fafeee17c588aa26cff014</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ea)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>_usbd_control_in</name>
      <anchorfile>usb__control_8c.html</anchorfile>
      <anchor>a322b01bbbac6fb3c095d4bb6fc54184d</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ea)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb_f103.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/usb/</path>
    <filename>usb__f103_8c</filename>
    <member kind="function" static="yes">
      <type>static usbd_device *</type>
      <name>stm32f103_usbd_init</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a4f1968b72b13c780a719652fdd6a8b8a</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>stm32f103_set_address</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a0228f4fa04e7c7d1ccf05c9c7ed106dc</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>stm32f103_ep_setup</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>af09955f524ada7d17da30b03553bb6ab</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t type, uint16_t max_size, void(*callback)(usbd_device *usbd_dev, uint8_t ep))</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>stm32f103_endpoints_reset</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a97482ce8f4274239adfd79099109bdc3</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>stm32f103_ep_stall_set</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>ade798f44c10e74d77cec089cff0a0f64</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t stall)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint8_t</type>
      <name>stm32f103_ep_stall_get</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a4c524b870ca913f26edbbcddece37975</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>stm32f103_ep_nak_set</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a95ae6b3af911e67ce72b486066c37d55</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t nak)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint16_t</type>
      <name>stm32f103_ep_write_packet</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>ad463af6a14ee4bbdd294a23369f5372d</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, const void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint16_t</type>
      <name>stm32f103_ep_read_packet</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a4890673806855a275ed1c467d4034d7f</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>stm32f103_poll</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a755f83aa414568675f88d549675d8d67</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>usb_set_ep_rx_bufsize</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a5f29b274550ac5fd30f1d2eb90548fa8</anchor>
      <arglist>(usbd_device *dev, uint8_t ep, uint32_t size)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>usb_copy_to_pm</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>ab1ab2d5108d7a424fc7d707fe2eaa117</anchor>
      <arglist>(volatile void *vPM, const void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>usb_copy_from_pm</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a65002ec3c6d1073785ef61846bb755ec</anchor>
      <arglist>(void *buf, const volatile void *vPM, uint16_t len)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static uint8_t</type>
      <name>force_nak</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a2e43380a97ab967fc166330fe9cae37b</anchor>
      <arglist>[8]</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static struct _usbd_device</type>
      <name>usbd_dev</name>
      <anchorfile>usb__f103_8c.html</anchorfile>
      <anchor>a4ea025cca3cc88b8f4000413d174db1c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct _usbd_driver</type>
      <name>stm32f103_usb_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaf75394477d6005c2032c0d2aebb7a9a6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb_f107.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/usb/</path>
    <filename>usb__f107_8c</filename>
    <member kind="function" static="yes">
      <type>static usbd_device *</type>
      <name>stm32f107_usbd_init</name>
      <anchorfile>usb__f107_8c.html</anchorfile>
      <anchor>a1d28133aed4b8b0e1d19705324cbd004</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static struct _usbd_device</type>
      <name>usbd_dev</name>
      <anchorfile>usb__f107_8c.html</anchorfile>
      <anchor>a4ea025cca3cc88b8f4000413d174db1c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct _usbd_driver</type>
      <name>stm32f107_usb_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga5adac4ccb88c72946fa692670d0bfc16</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb_f207.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/usb/</path>
    <filename>usb__f207_8c</filename>
    <member kind="function" static="yes">
      <type>static usbd_device *</type>
      <name>stm32f207_usbd_init</name>
      <anchorfile>usb__f207_8c.html</anchorfile>
      <anchor>a68c4ceb309e31e2895e61e4248adc958</anchor>
      <arglist>(void)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static struct _usbd_device</type>
      <name>usbd_dev</name>
      <anchorfile>usb__f207_8c.html</anchorfile>
      <anchor>a4ea025cca3cc88b8f4000413d174db1c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct _usbd_driver</type>
      <name>stm32f207_usb_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga2170186d661d2005c6b18c06d7f49f12</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb_fx07_common.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/usb/</path>
    <filename>usb__fx07__common_8c</filename>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_set_address</name>
      <anchorfile>usb__fx07__common_8c.html</anchorfile>
      <anchor>a13ff93197dbe1e4a33c382718ab936a4</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_ep_setup</name>
      <anchorfile>usb__fx07__common_8c.html</anchorfile>
      <anchor>ae2625812391f4d41bfce2007187a68bc</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t type, uint16_t max_size, void(*callback)(usbd_device *usbd_dev, uint8_t ep))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_endpoints_reset</name>
      <anchorfile>usb__fx07__common_8c.html</anchorfile>
      <anchor>a5f11ba15e82378b9c64ffe3421adf464</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_ep_stall_set</name>
      <anchorfile>usb__fx07__common_8c.html</anchorfile>
      <anchor>a0e2ff8af2add12281fffad6aa8bb44a1</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t stall)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>stm32fx07_ep_stall_get</name>
      <anchorfile>usb__fx07__common_8c.html</anchorfile>
      <anchor>a4f8070cf295dc3c253c384724b9971d6</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_ep_nak_set</name>
      <anchorfile>usb__fx07__common_8c.html</anchorfile>
      <anchor>a5be6f7b00bfb0d98dd96e54b798e376e</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t nak)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>stm32fx07_ep_write_packet</name>
      <anchorfile>usb__fx07__common_8c.html</anchorfile>
      <anchor>ae1410725ae62247feeb257c608f43542</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, const void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>stm32fx07_ep_read_packet</name>
      <anchorfile>usb__fx07__common_8c.html</anchorfile>
      <anchor>aa263b30fa161295b25907aed2167503a</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_poll</name>
      <anchorfile>usb__fx07__common_8c.html</anchorfile>
      <anchor>a6ac887edf6cf4f3bc7233df12ba9eefd</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_disconnect</name>
      <anchorfile>usb__fx07__common_8c.html</anchorfile>
      <anchor>a780f34c82133eae2d2e02a5105d7e70c</anchor>
      <arglist>(usbd_device *usbd_dev, bool disconnected)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb_fx07_common.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/usb/</path>
    <filename>usb__fx07__common_8h</filename>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_set_address</name>
      <anchorfile>usb__fx07__common_8h.html</anchorfile>
      <anchor>a13ff93197dbe1e4a33c382718ab936a4</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_ep_setup</name>
      <anchorfile>usb__fx07__common_8h.html</anchorfile>
      <anchor>ae2625812391f4d41bfce2007187a68bc</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t type, uint16_t max_size, void(*callback)(usbd_device *usbd_dev, uint8_t ep))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_endpoints_reset</name>
      <anchorfile>usb__fx07__common_8h.html</anchorfile>
      <anchor>a5f11ba15e82378b9c64ffe3421adf464</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_ep_stall_set</name>
      <anchorfile>usb__fx07__common_8h.html</anchorfile>
      <anchor>a0e2ff8af2add12281fffad6aa8bb44a1</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t stall)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>stm32fx07_ep_stall_get</name>
      <anchorfile>usb__fx07__common_8h.html</anchorfile>
      <anchor>a4f8070cf295dc3c253c384724b9971d6</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_ep_nak_set</name>
      <anchorfile>usb__fx07__common_8h.html</anchorfile>
      <anchor>a5be6f7b00bfb0d98dd96e54b798e376e</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t nak)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>stm32fx07_ep_write_packet</name>
      <anchorfile>usb__fx07__common_8h.html</anchorfile>
      <anchor>ae1410725ae62247feeb257c608f43542</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, const void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>stm32fx07_ep_read_packet</name>
      <anchorfile>usb__fx07__common_8h.html</anchorfile>
      <anchor>aa263b30fa161295b25907aed2167503a</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_poll</name>
      <anchorfile>usb__fx07__common_8h.html</anchorfile>
      <anchor>a6ac887edf6cf4f3bc7233df12ba9eefd</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stm32fx07_disconnect</name>
      <anchorfile>usb__fx07__common_8h.html</anchorfile>
      <anchor>a780f34c82133eae2d2e02a5105d7e70c</anchor>
      <arglist>(usbd_device *usbd_dev, bool disconnected)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb_msc.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/usb/</path>
    <filename>usb__msc_8c</filename>
    <class kind="struct">usb_msc_cbw</class>
    <class kind="struct">usb_msc_csw</class>
    <class kind="struct">sbc_sense_info</class>
    <class kind="struct">usb_msc_trans</class>
    <class kind="struct">_usbd_mass_storage</class>
    <member kind="enumeration">
      <type></type>
      <name>sbc_sense_key</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_NO_SENSE</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20caaa74b9a274f2fbbf372235e2d9364f9e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_RECOVERED_ERROR</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20caa312059808dae88e8ade4cd46a475ef8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_NOT_READY</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20cae8940c3371f429869cb5bbc8febee89e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_MEDIUM_ERROR</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20cad200d8d5ef1c6d93749294ce66e495b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_HARDWARE_ERROR</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20ca3d5a1b0b01fa2769c349eef7a6150083</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_ILLEGAL_REQUEST</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20cab6292ba727cf98e82a5530f40bca23a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_UNIT_ATTENTION</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20caa2d1609b26c17accb683ff05e2af2c9c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_DATA_PROTECT</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20cadad2cee9d79bdc561e213cd89cfc5c4a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_BLANK_CHECK</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20ca55a9626569077335b38b3df37801f412</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_VENDOR_SPECIFIC</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20caba4d8fab974ef4229b2ed5574474cffb</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_COPY_ABORTED</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20cafd62f8485d702d4475aa6752372ca3ad</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_ABORTED_COMMAND</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20ca30d0d91530fd75fb9e913328dfb3bca9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_VOLUME_OVERFLOW</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20cab074a7fc1941f268a56266fa803f6363</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_SENSE_KEY_MISCOMPARE</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af26abd40ddc343bd44bf4c12c93cb20ca6a8b413783e99c610987a893f8710d36</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>sbc_asc</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_NO_ADDITIONAL_SENSE_INFORMATION</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958ad6892f4955361f9b8ea37ab7dec24adf</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_PERIPHERAL_DEVICE_WRITE_FAULT</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958a41bdc2d78f61526d1a435390466f5bea</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_LOGICAL_UNIT_NOT_READY</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958afc7b9ee0d5c63434a065893f12bf2509</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_UNRECOVERED_READ_ERROR</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958af94805113caba66e7f69d0d0efcdd90a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_INVALID_COMMAND_OPERATION_CODE</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958a4ac550ebdaa7d2d2b2e874845dde9bf2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_LBA_OUT_OF_RANGE</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958a94670e961a4b34f918694211799fa085</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_INVALID_FIELD_IN_CDB</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958ae671358697a0f3fc7ae869afe3765281</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_WRITE_PROTECTED</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958af6d3b29c303410899fa552f3d5ab6413</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_NOT_READY_TO_READY_CHANGE</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958a3aa858eb504fe4cac3aa93921749b270</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_FORMAT_ERROR</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958a0eba7d6a516888c49d2c0379dd4f655e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASC_MEDIUM_NOT_PRESENT</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a77f0039858eb3e65043ab38cee89b958a6122693543dfa6a6793fdc835050fbb2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>sbc_ascq</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>ab27651977836aae95470be25085abb23</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASCQ_NA</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>ab27651977836aae95470be25085abb23affc75b07e3e5040bea2fe3ca1c418716</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASCQ_FORMAT_COMMAND_FAILED</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>ab27651977836aae95470be25085abb23aa7e184eb37b0909faa4a2fd302ad928a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASCQ_INITIALIZING_COMMAND_REQUIRED</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>ab27651977836aae95470be25085abb23a577b28cd3a92592b4abd7bb4ffd057bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SBC_ASCQ_OPERATION_IN_PROGRESS</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>ab27651977836aae95470be25085abb23ac56f5655000705ee5d5578d1c82c3025</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>trans_event</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>adabc5b9359b701c26304af43ea9e6954</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>EVENT_CBW_VALID</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>adabc5b9359b701c26304af43ea9e6954a439faeac7135a05e3efa6870939895fb</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>EVENT_NEED_STATUS</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>adabc5b9359b701c26304af43ea9e6954a2cbd45952a590a6297e91cfa52aebd9f</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>struct usb_msc_cbw</type>
      <name>__attribute__</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af1136d3ad4b67e5f06904fde6c067377</anchor>
      <arglist>((packed))</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>set_sbc_status</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a285558a874e454328ae7cba96306b253</anchor>
      <arglist>(usbd_mass_storage *ms, enum sbc_sense_key key, enum sbc_asc asc, enum sbc_ascq ascq)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>set_sbc_status_good</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>acea2b1d4fda2777c7729104dfb3109ba</anchor>
      <arglist>(usbd_mass_storage *ms)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint8_t *</type>
      <name>get_cbw_buf</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>ac5b311ee22d1d86e3bf5d93cd23d78d7</anchor>
      <arglist>(struct usb_msc_trans *trans)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>scsi_read_6</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a700c7b32d7db1ab7d9a55fb750cfed1a</anchor>
      <arglist>(usbd_mass_storage *ms, struct usb_msc_trans *trans, enum trans_event event)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>scsi_write_6</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af0e6535bbde2e9a00c6e4c3c77d34540</anchor>
      <arglist>(usbd_mass_storage *ms, struct usb_msc_trans *trans, enum trans_event event)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>scsi_write_10</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a67ab7f906e09e9fc149b8fa647a91d2f</anchor>
      <arglist>(usbd_mass_storage *ms, struct usb_msc_trans *trans, enum trans_event event)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>scsi_read_10</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>aa9340553bfc7682037cce665df163506</anchor>
      <arglist>(usbd_mass_storage *ms, struct usb_msc_trans *trans, enum trans_event event)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>scsi_read_capacity</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a40e07cb7889a673d30d93cae4187a5ec</anchor>
      <arglist>(usbd_mass_storage *ms, struct usb_msc_trans *trans, enum trans_event event)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>scsi_format_unit</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a0ba002041133ce594072c4669a888423</anchor>
      <arglist>(usbd_mass_storage *ms, struct usb_msc_trans *trans, enum trans_event event)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>scsi_request_sense</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a4db898a25d3e9e5a3187c6489a8fcd9d</anchor>
      <arglist>(usbd_mass_storage *ms, struct usb_msc_trans *trans, enum trans_event event)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>scsi_mode_sense_6</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a3a155b5971bdfabd70a4d964fd175da1</anchor>
      <arglist>(usbd_mass_storage *ms, struct usb_msc_trans *trans, enum trans_event event)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>scsi_inquiry</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a85cd7451c1bfc24383b60bd8b53073a1</anchor>
      <arglist>(usbd_mass_storage *ms, struct usb_msc_trans *trans, enum trans_event event)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>scsi_command</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>ad834e80e08a1fc52cb6c2a7e4c60e7c4</anchor>
      <arglist>(usbd_mass_storage *ms, struct usb_msc_trans *trans, enum trans_event event)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>msc_data_rx_cb</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a41a8c24958254f6628787f0a97befc1c</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ep)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>msc_data_tx_cb</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a88700274691bf361f44a0e3b5703ec9f</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ep)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>msc_control_request</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a7597112b5390c72fe40dd626ba1c5df6</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void(**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>msc_set_config</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a226667566a4c9460bc577aa9c72a14b6</anchor>
      <arglist>(usbd_device *usbd_dev, uint16_t wValue)</arglist>
    </member>
    <member kind="function">
      <type>usbd_mass_storage *</type>
      <name>usb_msc_init</name>
      <anchorfile>group__usb__msc.html</anchorfile>
      <anchor>ga5e6959c3ac6ff4efab4fd3b59353f497</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ep_in, uint8_t ep_in_size, uint8_t ep_out, uint8_t ep_out_size, const char *vendor_id, const char *product_id, const char *product_revision_level, const uint32_t block_count, int(*read_block)(uint32_t lba, uint8_t *copy_to), int(*write_block)(uint32_t lba, const uint8_t *copy_from))</arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCBWSignature</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a25dd7ce6b77786d6f489dddb6e792922</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCBWTag</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a6490a217070a49d7b312b9ae89a7a13d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCBWDataTransferLength</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>ac2c0350283b9f39e01d9fcd06739d44b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmCBWFlags</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>adbcff2be14450de51d5d055a666050e9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bCBWLUN</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a1273a617fe2d3434199257559181652e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bCBWCBLength</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>afffd40cc51e64f280f734daa46463a2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>CBWCB</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>ac7174140836bafcce11192c288d618e7</anchor>
      <arglist>[16]</arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCSWSignature</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a00618cfef87f1f50506029b4b9ba5b26</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCSWTag</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a6d632dc494a1ecab945e85f7cf2ebebe</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCSWDataResidue</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a0828bc16e03cb92f0fc0eed899ed2afd</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bCSWStatus</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a1375947e53883d17bd586b6037626268</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct sbc_sense_info</type>
      <name>__attribute__</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a2584cca2f0bded88fb09df200f798fcd</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static usbd_mass_storage</type>
      <name>_mass_storage</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>af560a8ea5920227995c6f1741499fe6a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const uint8_t</type>
      <name>_spc3_inquiry_response</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a6b4bbdb252e4a7df3df345af3c352dae</anchor>
      <arglist>[36]</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static const uint8_t</type>
      <name>_spc3_request_sense</name>
      <anchorfile>usb__msc_8c.html</anchorfile>
      <anchor>a90b557bc0850bec9c2397368c6fd9424</anchor>
      <arglist>[18]</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb_private.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/usb/</path>
    <filename>usb__private_8h</filename>
    <class kind="struct">_usbd_device</class>
    <class kind="struct">_usbd_device::usb_control_state</class>
    <class kind="struct">_usbd_device::user_control_callback</class>
    <class kind="struct">_usbd_driver</class>
    <member kind="enumeration">
      <type></type>
      <name>_usbd_transaction</name>
      <anchorfile>group__usb__private__defines.html</anchorfile>
      <anchor>ga1cf959928954ac5fe754b82f75ce4994</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_TRANSACTION_IN</name>
      <anchorfile>group__usb__private__defines.html</anchorfile>
      <anchor>gga1cf959928954ac5fe754b82f75ce4994a9d186519ca162685e9cb4d3c17b8fdff</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_TRANSACTION_OUT</name>
      <anchorfile>group__usb__private__defines.html</anchorfile>
      <anchor>gga1cf959928954ac5fe754b82f75ce4994ab016c2caf10af8d5cd6adbb926a18c90</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_TRANSACTION_SETUP</name>
      <anchorfile>group__usb__private__defines.html</anchorfile>
      <anchor>gga1cf959928954ac5fe754b82f75ce4994a7439308c92f9f23a0e750befc7f94018</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>_usbd_control_in</name>
      <anchorfile>usb__private_8h.html</anchorfile>
      <anchor>a322b01bbbac6fb3c095d4bb6fc54184d</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ea)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>_usbd_control_out</name>
      <anchorfile>usb__private_8h.html</anchorfile>
      <anchor>a1a921e8785fafeee17c588aa26cff014</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ea)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>_usbd_control_setup</name>
      <anchorfile>usb__private_8h.html</anchorfile>
      <anchor>ad4bc495b7e757434644e1dece336b261</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ea)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>_usbd_standard_request_device</name>
      <anchorfile>usb__private_8h.html</anchorfile>
      <anchor>abd588664d0803d5bf7d4d263f34f2575</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>_usbd_standard_request_interface</name>
      <anchorfile>usb__private_8h.html</anchorfile>
      <anchor>aec01ef6ed66dcc2b50cc969b8be8f419</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>_usbd_standard_request_endpoint</name>
      <anchorfile>usb__private_8h.html</anchorfile>
      <anchor>aed9aa74939234f607611928534d3b1b9</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>_usbd_standard_request</name>
      <anchorfile>usb__private_8h.html</anchorfile>
      <anchor>a3347ca42a15acf3ae31d02126ba7d695</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>_usbd_reset</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga3bc8dd33500621a1e900c78a42dcaf27</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usb_standard.c</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/lib/usb/</path>
    <filename>usb__standard_8c</filename>
    <member kind="function">
      <type>int</type>
      <name>usbd_register_set_config_callback</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>gab61ace3fbfd23ca3bc3afcee6c231d05</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(usbd_device *usbd_dev, uint16_t wValue))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_set_altsetting_callback</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga21d5f43101d1e56baf0d9f664401e506</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue))</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint16_t</type>
      <name>build_config_descriptor</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>gaf8a21c66ec88535a258872791c468098</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t index, uint8_t *buf, uint16_t len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_descriptor_type</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga2df65bb0dcba3643cde420b3fdc08ff0</anchor>
      <arglist>(uint16_t wValue)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_descriptor_index</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>gaf52328d55b34702e49cb14ec9005ca19</anchor>
      <arglist>(uint16_t wValue)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_get_descriptor</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>gad9327961f96b6203ac232d69d53afcf6</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_set_address</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga2e39a66a58cc636b01e3d90cb70c7625</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_set_configuration</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga11f2ca9802883c0ff1dc312799e7250d</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_get_configuration</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga430326cdf658101dd4b02d52ebb0cf3c</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_set_interface</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga137aae06450b52a0c22f7ffa6fba5ba0</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_get_interface</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga22c09fa8b89c6d0cb46e290cfecdf33f</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_device_get_status</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga2c512d44bd72a83d9b438859ab3ee352</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_interface_get_status</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga3af045144aef2572263f2ec0a2fdbef9</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_endpoint_get_status</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga4cfdd2484f7002da2cc98eea1e6a69e9</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_endpoint_stall</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>gad24d6daf25ffd5539979b9ea1648ec6f</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_endpoint_unstall</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga9714d02e4367d72d66f7d71abe10a70f</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>_usbd_standard_request_device</name>
      <anchorfile>usb__standard_8c.html</anchorfile>
      <anchor>abd588664d0803d5bf7d4d263f34f2575</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>_usbd_standard_request_interface</name>
      <anchorfile>usb__standard_8c.html</anchorfile>
      <anchor>aec01ef6ed66dcc2b50cc969b8be8f419</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>_usbd_standard_request_endpoint</name>
      <anchorfile>usb__standard_8c.html</anchorfile>
      <anchor>aed9aa74939234f607611928534d3b1b9</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>_usbd_standard_request</name>
      <anchorfile>usb__standard_8c.html</anchorfile>
      <anchor>a3347ca42a15acf3ae31d02126ba7d695</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usbd.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/usb/</path>
    <filename>usbd_8h</filename>
    <member kind="typedef">
      <type>struct _usbd_driver</type>
      <name>usbd_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaf89d061a4e982ccc705dbf1fc7f14fca</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>struct _usbd_device</type>
      <name>usbd_device</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga398cbe724daaf03b101d88163e7c1fce</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>int(*</type>
      <name>usbd_control_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaf05abc6fde0ddaf3b630222a8422579f</anchor>
      <arglist>)(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void(**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))</arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>usbd_request_return_codes</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gae09dc4bce47d0848ddacbd8b08ef76a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USBD_REQ_NOTSUPP</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ggae09dc4bce47d0848ddacbd8b08ef76a7ae6fb6de05148b303504d1f2855534495</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USBD_REQ_HANDLED</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ggae09dc4bce47d0848ddacbd8b08ef76a7ae44b9555863f8ecd5415bef01cb5779f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USBD_REQ_NEXT_CALLBACK</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ggae09dc4bce47d0848ddacbd8b08ef76a7a2de82a2c4708a6cd78384a3362e86d7f</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>usbd_device *</type>
      <name>usbd_init</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga8bbc488106ca72c2b550c1c9f89d5748</anchor>
      <arglist>(const usbd_driver *driver, const struct usb_device_descriptor *dev, const struct usb_config_descriptor *conf, const char **strings, int num_strings, uint8_t *control_buffer, uint16_t control_buffer_size)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_reset_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga2cdd6f2cd24e7e222b2add6e08283a7e</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_suspend_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga064f9d058427b1fb6f40b35e03a8cd5b</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_resume_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga004c77a73e4dcd6216678925f3fa1f41</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_sof_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gad8e688f14cac90c4933954e3ebbfc3b7</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>usbd_register_control_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga86f80b7593ee05d364fd791896213e38</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t type, uint8_t type_mask, usbd_control_callback callback)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>usbd_register_set_config_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gab61ace3fbfd23ca3bc3afcee6c231d05</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(usbd_device *usbd_dev, uint16_t wValue))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_set_altsetting_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga21d5f43101d1e56baf0d9f664401e506</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_poll</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga94eba5aff7befd8009cea2f2abb43134</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_disconnect</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaef20aadd330e4bb9e97635eeb751d8f9</anchor>
      <arglist>(usbd_device *usbd_dev, bool disconnected)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_setup</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga888110ac1598406bb834b494c8db7f6c</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t type, uint16_t max_size, void(*callback)(usbd_device *usbd_dev, uint8_t ep))</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>usbd_ep_write_packet</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaa5b245bdd9757cdc79d1fe721ebcf03f</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, const void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>usbd_ep_read_packet</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gabd42dc86361dbf9c639fd883ada1f572</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_stall_set</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga6d16be856512df298e40dfa31a216172</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t stall)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>usbd_ep_stall_get</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga396b34ccbe5bf6718417ef3ee0c3b788</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_nak_set</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga2f2a7dac5031fd8d1b5eb076d3f24b42</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t nak)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_cable_connect</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga3c83b9f71370e276c4550b25f013d833</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t on)</arglist>
    </member>
    <member kind="variable">
      <type>BEGIN_DECLS enum usbd_request_return_codes</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga2530c9cbb5ab3be81daa4e988c2ce579</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const usbd_driver</type>
      <name>stm32f103_usb_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaf75394477d6005c2032c0d2aebb7a9a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const usbd_driver</type>
      <name>stm32f107_usb_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga5adac4ccb88c72946fa692670d0bfc16</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const usbd_driver</type>
      <name>stm32f207_usb_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga2170186d661d2005c6b18c06d7f49f12</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>usbstd.h</name>
    <path>/home/aghosh01/TARS_codebase/libopencm3/include/libopencm3/usb/</path>
    <filename>usbstd_8h</filename>
    <class kind="struct">usb_setup_data</class>
    <class kind="struct">usb_device_descriptor</class>
    <class kind="struct">usb_device_qualifier_descriptor</class>
    <class kind="struct">usb_config_descriptor</class>
    <class kind="struct">usb_config_descriptor::usb_interface</class>
    <class kind="struct">usb_interface</class>
    <class kind="struct">usb_interface_descriptor</class>
    <class kind="struct">usb_endpoint_descriptor</class>
    <class kind="struct">usb_string_descriptor</class>
    <class kind="struct">usb_iface_assoc_descriptor</class>
    <member kind="enumeration">
      <type></type>
      <name>usb_language_id</name>
      <anchorfile>group__usb__type__defines.html</anchorfile>
      <anchor>gaca86007d66ae94421fd4f93c0b96b04a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_LANGID_ENGLISH_US</name>
      <anchorfile>group__usb__type__defines.html</anchorfile>
      <anchor>ggaca86007d66ae94421fd4f93c0b96b04aa069b825fab4882505aca430300ecc9a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>struct usb_setup_data</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__type__defines.html</anchorfile>
      <anchor>gaf45ea7b5ea10239fbbd9377f7faae445</anchor>
      <arglist>((packed))</arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmRequestType</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>abea120ea756eb0abdb8aafb8713f61f4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bRequest</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a1e7be110861bd4ff75c86d5214f31543</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wValue</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a7383d0165ad8eab48aa239c005199121</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wIndex</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>adc578ce35d89d52deba61ee6f312f177</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wLength</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a496c03443b177fd2e6c93616064d2934</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a1deca1f1d6e5815b290e6e1015bce5b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a67d7027b9eb9ed268d28e84fbc675707</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdUSB</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a8f2450b83d08617906b5b63640f559af</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDeviceClass</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>ab8e1b1fca77a607c39bba2278623d4f4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDeviceSubClass</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a260eec0b4e7279e6ecae808ca34c2604</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDeviceProtocol</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a44e204b5f7158f4048edcb944351ce27</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bMaxPacketSize0</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a93b2c926b005917cdf2b5e7e4b3f80a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>idVendor</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>ab393e07556e3858fed36ed2e260d4b4c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>idProduct</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a2cf29745398a958349d784b1911c6b2b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdDevice</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a4ac87bf0a9ee1a9e0eec6d50a8b0e3f9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iManufacturer</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a7ee6ae41b483f3c49d1ddb39a83a8a43</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iProduct</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a77c33850b90ff7617243c40ce61d9168</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iSerialNumber</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>aeb129fc0d41494f69369da70d73eac11</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumConfigurations</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>abde4f60e84b2d9b10c27aa23bae62595</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bReserved</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>ab8303f57004812636de0daff446b975a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wTotalLength</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a866c5d9bb8a60f25805d37b7a5b5f9e8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumInterfaces</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a1ad144333e75b0d867e72f8a337cd510</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bConfigurationValue</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>aecb9ae30bca0b044d2941120f21c2b29</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iConfiguration</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a5ad4440b4eb1935d66b154d274f8a272</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmAttributes</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>ae7c63b4cda39953db5710012a5d91c6f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bMaxPower</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>ad3ac74420252d7a6a03e1e9d7b481f9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct usb_interface *</type>
      <name>interface</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>ab658b15528f33b23d2f67ef53ad95648</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterfaceNumber</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a3d66d88c5e928462aef62633449f1eb0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bAlternateSetting</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a4cdd7aab185365e51a1e722f43073b8c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumEndpoints</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>acec2e7d68271baace354a0539f3d7e3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterfaceClass</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>ae899427763e325f03241afc69a6e1ea0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterfaceSubClass</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a3b3919d1203cd42550d77287f7585595</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterfaceProtocol</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a6872c4ec51fd061e2955f84d97bea2a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iInterface</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a2e4535908d90f195c6347a91e15b544d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct usb_endpoint_descriptor *</type>
      <name>endpoint</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>ad9632438abef761b213b1c19f02983ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const void *</type>
      <name>extra</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a28cfbca8dce621e74d9da0036b1f5789</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>extralen</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>aceea3d2ad4e8495c4356caa0afc8d590</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bEndpointAddress</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a527afb0f25cdd442d8d32c7df8e18c45</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wMaxPacketSize</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>abcc8edb1d5094ce6a16b42c1a7ae67d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterval</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>aff92277eea8536f7de1a88d818c91b9e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wData</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a6a2ad4cf42839c148443411bbd9d8805</anchor>
      <arglist>[]</arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFirstInterface</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a85323ed61eb948bdea147d67e01ddc2d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterfaceCount</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a9674bda677f9898106ec2b6fe5f5723d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionClass</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a1ed25bc3320ca90e4d3acb4410aea704</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionSubClass</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>ab25f749c091ce99b6868c949c4e45bc3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionProtocol</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>a2b3b905ab34e6e28b57c3ddc51b3487c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iFunction</name>
      <anchorfile>usbstd_8h.html</anchorfile>
      <anchor>ab84d4b3bced6c09c62350d56fd69d226</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>enum usb_language_id</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__type__defines.html</anchorfile>
      <anchor>ga993287986228cc24f80bfdf807896554</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>_usbd_device</name>
    <filename>struct__usbd__device.html</filename>
    <class kind="struct">_usbd_device::usb_control_state</class>
    <class kind="struct">_usbd_device::user_control_callback</class>
    <member kind="variable">
      <type>const struct usb_device_descriptor *</type>
      <name>desc</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>abf8c97de9264969c27b342cc569a8ed9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct usb_config_descriptor *</type>
      <name>config</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>ae60b7e4cb2c404b6e86fdf6bbafdb5d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const char **</type>
      <name>strings</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a76f748749894cc0316e3e95ea7a8be27</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>num_strings</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a3fe1f0382e2bc3e687152bc47b7e4342</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t *</type>
      <name>ctrl_buf</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>ac28d5ce184bf3812d6e85be62ce27ea4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>ctrl_buf_len</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a10d573425d5619bf41cf70c0e3bd298d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>current_address</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>acb76927d8afd9bf487d2f8f645e2ee0d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>current_config</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a72a6261b52abbe62672c230967268d59</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>pm_top</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a46873de4e0e3671bb54bd6f7f7f75acd</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>user_callback_reset</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a171474ef610c019e11546b98b2cea769</anchor>
      <arglist>)(void)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>user_callback_suspend</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>ae19323e910fc01e5670c46448e5754b7</anchor>
      <arglist>)(void)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>user_callback_resume</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>af22d910c0ba8b4565a7fd7d92f15bc18</anchor>
      <arglist>)(void)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>user_callback_sof</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>afed1960c9106fe7b483390b83cd3bf29</anchor>
      <arglist>)(void)</arglist>
    </member>
    <member kind="variable">
      <type>struct _usbd_device::usb_control_state</type>
      <name>control_state</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a4c28e71881bec82fc45d83860c874816</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct _usbd_device::user_control_callback</type>
      <name>user_control_callback</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a31606577bf4ff006a56ef0f8d57a29d4</anchor>
      <arglist>[MAX_USER_CONTROL_CALLBACK]</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>user_callback_ctr</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a6e82880b623706d1dd345e54fed03445</anchor>
      <arglist>[8][3])(usbd_device *usbd_dev, uint8_t ea)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>user_callback_set_config</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a87d8f0db30486e8a4d2369d95a826e3f</anchor>
      <arglist>[MAX_USER_SET_CONFIG_CALLBACK])(usbd_device *usbd_dev, uint16_t wValue)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>user_callback_set_altsetting</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>affa9de68619473e97bdd4fd669f920ce</anchor>
      <arglist>)(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue)</arglist>
    </member>
    <member kind="variable">
      <type>const struct _usbd_driver *</type>
      <name>driver</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>ad780343e8e2776ce99b9dd302b77c2e2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>fifo_mem_top</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a212245da83ffeb58ceb0b35df4482f82</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>fifo_mem_top_ep0</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>ad0879153fd5676dda54f0c122420af2e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>force_nak</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a1ed037a5f085c91a1ee3872a179e6383</anchor>
      <arglist>[4]</arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>doeptsiz</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a56153dea109cdf0eb2b0871bc1ea31ae</anchor>
      <arglist>[4]</arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>rxbcnt</name>
      <anchorfile>struct__usbd__device.html</anchorfile>
      <anchor>a55283552010d81ebafde0c663f2d5443</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>_usbd_driver</name>
    <filename>struct__usbd__driver.html</filename>
    <member kind="variable">
      <type>usbd_device *(*</type>
      <name>init</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>a8b540de0300579f8f705c2837f09dbb6</anchor>
      <arglist>)(void)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>set_address</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>abeaa7024a5fd33f0dbaf54204df1fb4e</anchor>
      <arglist>)(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>ep_setup</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>aa19a8ec1092231647ad4f79badf1c3e4</anchor>
      <arglist>)(usbd_device *usbd_dev, uint8_t addr, uint8_t type, uint16_t max_size, void(*cb)(usbd_device *usbd_dev, uint8_t ep))</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>ep_reset</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>a9b525f7949a638c0a2666d43bf977c04</anchor>
      <arglist>)(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>ep_stall_set</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>a31a997c03d79427e01c5bb84de1a8de0</anchor>
      <arglist>)(usbd_device *usbd_dev, uint8_t addr, uint8_t stall)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>ep_nak_set</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>adcc98ae89ab0e1fd49c470d68e6ff084</anchor>
      <arglist>)(usbd_device *usbd_dev, uint8_t addr, uint8_t nak)</arglist>
    </member>
    <member kind="variable">
      <type>uint8_t(*</type>
      <name>ep_stall_get</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>ab684ea60d77fa8b9850ba5f1bf499606</anchor>
      <arglist>)(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="variable">
      <type>uint16_t(*</type>
      <name>ep_write_packet</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>a154dc02a1da75edc580ec29c32f67178</anchor>
      <arglist>)(usbd_device *usbd_dev, uint8_t addr, const void *buf, uint16_t len)</arglist>
    </member>
    <member kind="variable">
      <type>uint16_t(*</type>
      <name>ep_read_packet</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>aca16e36e7aa495b0692b39481a99f813</anchor>
      <arglist>)(usbd_device *usbd_dev, uint8_t addr, void *buf, uint16_t len)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>poll</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>a7948bbf8c66dd289dbe18c7f25813450</anchor>
      <arglist>)(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>disconnect</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>ae320bc137a239caab674916616701f62</anchor>
      <arglist>)(usbd_device *usbd_dev, bool disconnected)</arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>base_address</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>a1e10b198b59a86710df51cb76cff31fc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>set_address_before_status</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>aefdf9ebe191591530450f59fb63bc125</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>rx_fifo_size</name>
      <anchorfile>struct__usbd__driver.html</anchorfile>
      <anchor>a5fa81d06fb7e305b7218d5aa83322870</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>_usbd_mass_storage</name>
    <filename>struct__usbd__mass__storage.html</filename>
    <member kind="variable">
      <type>usbd_device *</type>
      <name>usbd_dev</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>a9fec9c49a4e06c7ede13d4b3418eea19</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>ep_in</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>a3fa979e1953045744f4122c0f015e07e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>ep_in_size</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>a0bf69fdf2e153ea290fbef99e938ad40</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>ep_out</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>a44699a30223c3a3303463b9ab88dbf23</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>ep_out_size</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>a44faed30916340b27dc001b109b1fc41</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const char *</type>
      <name>vendor_id</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>a6b3a5f238f939395de152667aac28895</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const char *</type>
      <name>product_id</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>abb7b48b3662cf537a233db009b74e338</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const char *</type>
      <name>product_revision_level</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>aaf80a7a068a13697e71898911ffb0ccd</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>block_count</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>a9ef6ea0554a88e4eaad6ba43d65390a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int(*</type>
      <name>read_block</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>ab778dc96ac270f90911f5f75fc354709</anchor>
      <arglist>)(uint32_t lba, uint8_t *copy_to)</arglist>
    </member>
    <member kind="variable">
      <type>int(*</type>
      <name>write_block</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>a8e668f6e72fcf56d656651ef6dba3495</anchor>
      <arglist>)(uint32_t lba, const uint8_t *copy_from)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>lock</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>afdf3f1e5d95bb17a59e07660a6b51db3</anchor>
      <arglist>)(void)</arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>unlock</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>af33eabd592737c53c0c40403996687bc</anchor>
      <arglist>)(void)</arglist>
    </member>
    <member kind="variable">
      <type>struct usb_msc_trans</type>
      <name>trans</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>a3de82f6be70b2a7e5b53ebf62768bda7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct sbc_sense_info</type>
      <name>sense</name>
      <anchorfile>struct__usbd__mass__storage.html</anchorfile>
      <anchor>a4b8f3d6aae1625ecc4a5ab39633d1cec</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>sbc_sense_info</name>
    <filename>structsbc__sense__info.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>key</name>
      <anchorfile>structsbc__sense__info.html</anchorfile>
      <anchor>a58afa6868e1f44924da858936bab08e0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>asc</name>
      <anchorfile>structsbc__sense__info.html</anchorfile>
      <anchor>a29db5c56445002e0e348eb696594b12d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>ascq</name>
      <anchorfile>structsbc__sense__info.html</anchorfile>
      <anchor>a243d7a69b9797252c6deacca33b9392c</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_audio_header_descriptor_body</name>
    <filename>structusb__audio__header__descriptor__body.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>baInterfaceNr</name>
      <anchorfile>structusb__audio__header__descriptor__body.html</anchorfile>
      <anchor>a976e3791523c27a2e413a18ccd159508</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_audio_header_descriptor_head</name>
    <filename>structusb__audio__header__descriptor__head.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__audio__header__descriptor__head.html</anchorfile>
      <anchor>aa0180630d1010542a44113f059a41e79</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__audio__header__descriptor__head.html</anchorfile>
      <anchor>a11555b5c543ad440c48e261498b50bb5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>structusb__audio__header__descriptor__head.html</anchorfile>
      <anchor>a7f68b758aaddfe1f26c6a4929442c020</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdADC</name>
      <anchorfile>structusb__audio__header__descriptor__head.html</anchorfile>
      <anchor>a2a1acfe9d8eb5b43d41aa89b46696206</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wTotalLength</name>
      <anchorfile>structusb__audio__header__descriptor__head.html</anchorfile>
      <anchor>a5a922f09d1805e1e2b8313bbe981a3d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>binCollection</name>
      <anchorfile>structusb__audio__header__descriptor__head.html</anchorfile>
      <anchor>a8f80de70c5c45029705dc4a53737eda5</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_cdc_acm_descriptor</name>
    <filename>structusb__cdc__acm__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionLength</name>
      <anchorfile>structusb__cdc__acm__descriptor.html</anchorfile>
      <anchor>a4adf866831c80578ce4565fcd5fbe0dc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__cdc__acm__descriptor.html</anchorfile>
      <anchor>ab5d042a59b24537a0d5b1987e0f7682f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>structusb__cdc__acm__descriptor.html</anchorfile>
      <anchor>aa3552211a6791f3c28d635af64db5974</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmCapabilities</name>
      <anchorfile>structusb__cdc__acm__descriptor.html</anchorfile>
      <anchor>a7867a3f1edeed55d0f7cf2215013c8a6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_cdc_call_management_descriptor</name>
    <filename>structusb__cdc__call__management__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionLength</name>
      <anchorfile>structusb__cdc__call__management__descriptor.html</anchorfile>
      <anchor>a2a600f7dbd6bf4345ea296cc44e44e3f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__cdc__call__management__descriptor.html</anchorfile>
      <anchor>a26926afbc81c6d76a0b214f982395c06</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>structusb__cdc__call__management__descriptor.html</anchorfile>
      <anchor>af0ae057cfc3b3ce11a331c71b55fb141</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmCapabilities</name>
      <anchorfile>structusb__cdc__call__management__descriptor.html</anchorfile>
      <anchor>a506f8aed40e0b216761481792847062e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDataInterface</name>
      <anchorfile>structusb__cdc__call__management__descriptor.html</anchorfile>
      <anchor>a30aac28b5b3f592971f4c2927475f53c</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_cdc_header_descriptor</name>
    <filename>structusb__cdc__header__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionLength</name>
      <anchorfile>structusb__cdc__header__descriptor.html</anchorfile>
      <anchor>ae52183bbe29c7a9cbe1a61f54f9b9697</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__cdc__header__descriptor.html</anchorfile>
      <anchor>aa888fac376fb790cabe6e103303bc124</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>structusb__cdc__header__descriptor.html</anchorfile>
      <anchor>a0d866395c42693d23c954afdd891c914</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdCDC</name>
      <anchorfile>structusb__cdc__header__descriptor.html</anchorfile>
      <anchor>afdcf54326cb9777f27ce491615358eb3</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_cdc_line_coding</name>
    <filename>structusb__cdc__line__coding.html</filename>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dwDTERate</name>
      <anchorfile>structusb__cdc__line__coding.html</anchorfile>
      <anchor>abba439cd113503e9223c9d61865319d2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bCharFormat</name>
      <anchorfile>structusb__cdc__line__coding.html</anchorfile>
      <anchor>aae309443995f9fe84f44a78bd866cc1a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bParityType</name>
      <anchorfile>structusb__cdc__line__coding.html</anchorfile>
      <anchor>ad1b91d6baa7411fa9f6341140f5ac216</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDataBits</name>
      <anchorfile>structusb__cdc__line__coding.html</anchorfile>
      <anchor>a81e0e896a0524b8db945e1a8b50668e6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_cdc_notification</name>
    <filename>structusb__cdc__notification.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmRequestType</name>
      <anchorfile>structusb__cdc__notification.html</anchorfile>
      <anchor>af7514a608ecfe83cd2a8481b7d62fe3b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNotification</name>
      <anchorfile>structusb__cdc__notification.html</anchorfile>
      <anchor>a9d2b9a84431e32f2380bda292eb7c5f2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wValue</name>
      <anchorfile>structusb__cdc__notification.html</anchorfile>
      <anchor>aefff96f443fd9d7ca49a897a9303d647</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wIndex</name>
      <anchorfile>structusb__cdc__notification.html</anchorfile>
      <anchor>a16a0d11a8158e6b5532d8aa24fc09da6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wLength</name>
      <anchorfile>structusb__cdc__notification.html</anchorfile>
      <anchor>a9fa99e9c9fd2abd8c3bc5fbcb04b290b</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_cdc_union_descriptor</name>
    <filename>structusb__cdc__union__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionLength</name>
      <anchorfile>structusb__cdc__union__descriptor.html</anchorfile>
      <anchor>ab46252197d3d587b19032f7b51e80e44</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__cdc__union__descriptor.html</anchorfile>
      <anchor>a12a70496baf70c311a5362d2955e9f78</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>structusb__cdc__union__descriptor.html</anchorfile>
      <anchor>aaae2bfc7d765011a39150b818b75b243</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bControlInterface</name>
      <anchorfile>structusb__cdc__union__descriptor.html</anchorfile>
      <anchor>a233201a145a5c0a0783d702a7bbd89b1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bSubordinateInterface0</name>
      <anchorfile>structusb__cdc__union__descriptor.html</anchorfile>
      <anchor>a4b409e885d9e7457a21918f5fff2b118</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_config_descriptor</name>
    <filename>structusb__config__descriptor.html</filename>
    <class kind="struct">usb_config_descriptor::usb_interface</class>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__config__descriptor.html</anchorfile>
      <anchor>aaa38eade89cca96312968258eaeade9d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__config__descriptor.html</anchorfile>
      <anchor>a69b0a0fe8f2f9152f299a333f352982e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wTotalLength</name>
      <anchorfile>structusb__config__descriptor.html</anchorfile>
      <anchor>a09fb7940f26366f7f0aedb496a18b97c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumInterfaces</name>
      <anchorfile>structusb__config__descriptor.html</anchorfile>
      <anchor>a712744e37c3759b311131278ec2cf074</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bConfigurationValue</name>
      <anchorfile>structusb__config__descriptor.html</anchorfile>
      <anchor>af50d14a1e97dab1a4483f03b969ca431</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iConfiguration</name>
      <anchorfile>structusb__config__descriptor.html</anchorfile>
      <anchor>ace6c0975162873b87744269bd188d297</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmAttributes</name>
      <anchorfile>structusb__config__descriptor.html</anchorfile>
      <anchor>aa46182c4bc520e2c60a83c84d80d1e23</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bMaxPower</name>
      <anchorfile>structusb__config__descriptor.html</anchorfile>
      <anchor>a228c71b864eef8c5b997ad49b996ae9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct usb_config_descriptor::usb_interface *</type>
      <name>interface</name>
      <anchorfile>structusb__config__descriptor.html</anchorfile>
      <anchor>a93d0c4e3b6c5a9d6d84e6b769dbe1d2a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>_usbd_device::usb_control_state</name>
    <filename>struct__usbd__device_1_1usb__control__state.html</filename>
    <member kind="enumvalue">
      <name>IDLE</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56aa9114b048bdf93917cd2e2499e0232e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STALLED</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56adbdd8c4000539bee13614e666ce87819</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DATA_IN</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56a6d10e42fd61e631bfc28dc000945a68b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>LAST_DATA_IN</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56ad46e499dfabd69eeb4951d6f37350f78</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATUS_IN</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56a2b8f315a1c77155a8eb55bb2920c387f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DATA_OUT</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56aab55ae38c1bd603e61945595ee71dd27</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>LAST_DATA_OUT</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56ac4102e7aece07c0cd5679b2b3ff2f953</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATUS_OUT</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56a60e953b1e72ce2e6aa3fe204dabfdae3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>IDLE</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56aa9114b048bdf93917cd2e2499e0232e1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STALLED</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56adbdd8c4000539bee13614e666ce87819</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DATA_IN</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56a6d10e42fd61e631bfc28dc000945a68b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>LAST_DATA_IN</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56ad46e499dfabd69eeb4951d6f37350f78</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATUS_IN</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56a2b8f315a1c77155a8eb55bb2920c387f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DATA_OUT</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56aab55ae38c1bd603e61945595ee71dd27</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>LAST_DATA_OUT</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56ac4102e7aece07c0cd5679b2b3ff2f953</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATUS_OUT</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a62bcb381cd430830ed90561033b84f56a60e953b1e72ce2e6aa3fe204dabfdae3</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>struct usb_setup_data req</type>
      <name>__attribute__</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a2dfd02e13d09c8452ef9e9d346465027</anchor>
      <arglist>((aligned(4)))</arglist>
    </member>
    <member kind="variable">
      <type>enum _usbd_device::usb_control_state::@2</type>
      <name>state</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>ad23533845acfed1371d89d2703610d9e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t *</type>
      <name>ctrl_buf</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>a261f6c057a987f0ad538ecc8ad4d49b3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>ctrl_len</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>ad9e2cd348bdd9ca9135cedb7679ed916</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>void(*</type>
      <name>complete</name>
      <anchorfile>struct__usbd__device_1_1usb__control__state.html</anchorfile>
      <anchor>ab344b1656a1e8954d6b0a2c13050e7c0</anchor>
      <arglist>)(usbd_device *usbd_dev, struct usb_setup_data *req)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_device_descriptor</name>
    <filename>structusb__device__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>af3f980ad55af3fd6c222a8802996ab63</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>ad1ba08da6ad5b6023f1d0d1461daab7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdUSB</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>aa400edb6c3183d4922411cdaf980b84e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDeviceClass</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>aa657267e1d9762b7d2ed3eb60a78d9ad</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDeviceSubClass</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>aecfbe730bc3eeccc9c4b5fd17f5f3c3c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDeviceProtocol</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>a56829af76e57a6ea4fc621b52a0664f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bMaxPacketSize0</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>ac7f47eb197506ac5c555bb4f2fe82d32</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>idVendor</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>a043126e48bebbab536540e44428b6b4f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>idProduct</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>a70d5ecc7bad486b8a8840d86aa151579</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdDevice</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>a41416aa4a49999d2f3f0f67bdc5fa7da</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iManufacturer</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>ad082330020575944b8471459b816cb40</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iProduct</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>acb90b91c59e65adbcc21949cf0f486f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iSerialNumber</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>a105d91b68091e61c9b13ea673fb98eaf</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumConfigurations</name>
      <anchorfile>structusb__device__descriptor.html</anchorfile>
      <anchor>a603204b0517e9ece9bc0d8476b2a7cdc</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_device_qualifier_descriptor</name>
    <filename>structusb__device__qualifier__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__device__qualifier__descriptor.html</anchorfile>
      <anchor>ab1bf7ff669706ee5370f90d72f2a6856</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__device__qualifier__descriptor.html</anchorfile>
      <anchor>a1547d76b3c2f7599d2621e4c73ef485d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdUSB</name>
      <anchorfile>structusb__device__qualifier__descriptor.html</anchorfile>
      <anchor>aabe610da20b0b4ccc9aefdabd3396bf1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDeviceClass</name>
      <anchorfile>structusb__device__qualifier__descriptor.html</anchorfile>
      <anchor>aeb3cbfe114aa775d5121a5580dafa16e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDeviceSubClass</name>
      <anchorfile>structusb__device__qualifier__descriptor.html</anchorfile>
      <anchor>a9335b5c4955f68463484b55eb480a1fb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDeviceProtocol</name>
      <anchorfile>structusb__device__qualifier__descriptor.html</anchorfile>
      <anchor>a9665e7509c611a4eaa0345d3f96ec707</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bMaxPacketSize0</name>
      <anchorfile>structusb__device__qualifier__descriptor.html</anchorfile>
      <anchor>ad065540e5d99fbd241b0712e601f62ee</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumConfigurations</name>
      <anchorfile>structusb__device__qualifier__descriptor.html</anchorfile>
      <anchor>a4022310ac474e11b6813af8c1f5d093f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bReserved</name>
      <anchorfile>structusb__device__qualifier__descriptor.html</anchorfile>
      <anchor>aa1557859c9220ae5fda7d32833845444</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_dfu_descriptor</name>
    <filename>structusb__dfu__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__dfu__descriptor.html</anchorfile>
      <anchor>aaae3ed6f5fc202453e5a295b25c391e0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__dfu__descriptor.html</anchorfile>
      <anchor>a545d551fdf9612abab9679ad9092fe21</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmAttributes</name>
      <anchorfile>structusb__dfu__descriptor.html</anchorfile>
      <anchor>a6eda776c2fa8ad9e92da5a9c0771c017</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wDetachTimeout</name>
      <anchorfile>structusb__dfu__descriptor.html</anchorfile>
      <anchor>a1c30b936c3e7ab6d94cd14464394a818</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wTransferSize</name>
      <anchorfile>structusb__dfu__descriptor.html</anchorfile>
      <anchor>a6f28ae6870039d8ba994a2aed1d6b8b9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdDFUVersion</name>
      <anchorfile>structusb__dfu__descriptor.html</anchorfile>
      <anchor>ac917f4ae3b4cda852e7e23a1378cc584</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_endpoint_descriptor</name>
    <filename>structusb__endpoint__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__endpoint__descriptor.html</anchorfile>
      <anchor>aac9ff1940e27153339becfd460c5b796</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__endpoint__descriptor.html</anchorfile>
      <anchor>aa6e24b3aa0c8130a4d39f2133ac5ed12</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bEndpointAddress</name>
      <anchorfile>structusb__endpoint__descriptor.html</anchorfile>
      <anchor>a5007051761d339b5bb27268a016947a5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmAttributes</name>
      <anchorfile>structusb__endpoint__descriptor.html</anchorfile>
      <anchor>a938a8fb34155743b6a68b64a79235538</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wMaxPacketSize</name>
      <anchorfile>structusb__endpoint__descriptor.html</anchorfile>
      <anchor>a0b665a4fbe5daf9ede7ae18d75e1c744</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterval</name>
      <anchorfile>structusb__endpoint__descriptor.html</anchorfile>
      <anchor>a8fbc4ec9a4230f70729db036cfc07c44</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const void *</type>
      <name>extra</name>
      <anchorfile>structusb__endpoint__descriptor.html</anchorfile>
      <anchor>a2b03a8c37d2c9ae35a59b6303b1beca5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>extralen</name>
      <anchorfile>structusb__endpoint__descriptor.html</anchorfile>
      <anchor>ad21f23975ef6f28490dd00ec44d3f404</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_hid_descriptor</name>
    <filename>structusb__hid__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__hid__descriptor.html</anchorfile>
      <anchor>adedc57a5e5e9873d1cb7d00d69bfe1c7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__hid__descriptor.html</anchorfile>
      <anchor>a01533992789ba6c7306e0f97c1c01765</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdHID</name>
      <anchorfile>structusb__hid__descriptor.html</anchorfile>
      <anchor>af5ba1299c6b42c862f2f3e478173218d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bCountryCode</name>
      <anchorfile>structusb__hid__descriptor.html</anchorfile>
      <anchor>a143eb3e58466e54c19c1b9ab8a31ac6f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumDescriptors</name>
      <anchorfile>structusb__hid__descriptor.html</anchorfile>
      <anchor>a0f0d0cf9739138f069273f284490e0d8</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_iface_assoc_descriptor</name>
    <filename>structusb__iface__assoc__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__iface__assoc__descriptor.html</anchorfile>
      <anchor>a433ae49c3f435e75e10f401ecb68a1b5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__iface__assoc__descriptor.html</anchorfile>
      <anchor>a34948c1c676d39917281d2e890f1455b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFirstInterface</name>
      <anchorfile>structusb__iface__assoc__descriptor.html</anchorfile>
      <anchor>a3c588c7fb5d595bfa9fff073050e5c1e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterfaceCount</name>
      <anchorfile>structusb__iface__assoc__descriptor.html</anchorfile>
      <anchor>a4434a744d8c537c636ca7bb5948107f9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionClass</name>
      <anchorfile>structusb__iface__assoc__descriptor.html</anchorfile>
      <anchor>aff5b2750abc7309555fd5264fc0e75be</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionSubClass</name>
      <anchorfile>structusb__iface__assoc__descriptor.html</anchorfile>
      <anchor>adfcaabfcb88f3965ad3e44641dc4add9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bFunctionProtocol</name>
      <anchorfile>structusb__iface__assoc__descriptor.html</anchorfile>
      <anchor>a6ab5223196a15f3f2c7f81fad1dce02f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iFunction</name>
      <anchorfile>structusb__iface__assoc__descriptor.html</anchorfile>
      <anchor>aa429753fa6dcc07d97979d28610e3313</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_interface</name>
    <filename>structusb__interface.html</filename>
    <member kind="variable">
      <type>uint8_t *</type>
      <name>cur_altsetting</name>
      <anchorfile>structusb__interface.html</anchorfile>
      <anchor>aa39c5be41340ee8c3fa71b7a0f7ba240</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>num_altsetting</name>
      <anchorfile>structusb__interface.html</anchorfile>
      <anchor>a9c5a9b0ae7b56dbad13ac3badc546661</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct usb_iface_assoc_descriptor *</type>
      <name>iface_assoc</name>
      <anchorfile>structusb__interface.html</anchorfile>
      <anchor>a52cfc78c2536053de0c4f1615a057f79</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct usb_interface_descriptor *</type>
      <name>altsetting</name>
      <anchorfile>structusb__interface.html</anchorfile>
      <anchor>a5fb4da13363349c7fa027c673ede9812</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_config_descriptor::usb_interface</name>
    <filename>structusb__config__descriptor_1_1usb__interface.html</filename>
    <member kind="variable">
      <type>uint8_t *</type>
      <name>cur_altsetting</name>
      <anchorfile>structusb__config__descriptor_1_1usb__interface.html</anchorfile>
      <anchor>a06451de61afb655e5e0fb756309248c4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>num_altsetting</name>
      <anchorfile>structusb__config__descriptor_1_1usb__interface.html</anchorfile>
      <anchor>aeeb7e4ed8d6a6ad1ec1eb0c18ee0f9e3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct usb_iface_assoc_descriptor *</type>
      <name>iface_assoc</name>
      <anchorfile>structusb__config__descriptor_1_1usb__interface.html</anchorfile>
      <anchor>ae868a8420fe236b5ee4c5dbd554a5b36</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct usb_interface_descriptor *</type>
      <name>altsetting</name>
      <anchorfile>structusb__config__descriptor_1_1usb__interface.html</anchorfile>
      <anchor>a44285d1e29ac0dad6d8fd41afe1b2d8a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_interface_descriptor</name>
    <filename>structusb__interface__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>a7959ad865fb5527dba879ded533f5976</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>a757761276c9053ed877cee89cba167ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterfaceNumber</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>aea613784e25f453d059635a8d292a249</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bAlternateSetting</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>aacc6f1a4e77883f97ffb0453e41c5b10</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumEndpoints</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>a1fa779eda460fd7f670f7e1adeceed94</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterfaceClass</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>acccd957d6f023f3d4701abd491c45cf6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterfaceSubClass</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>ac807fb07b6b89886836ad5c1253dae82</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInterfaceProtocol</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>a177ebd30941b905658e1f89654d320c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iInterface</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>a63008abbea76cc397836554747386055</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const struct usb_endpoint_descriptor *</type>
      <name>endpoint</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>a241f94bbfcabf012866de8a34a46c10d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const void *</type>
      <name>extra</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>ae3aefe51473859b0a822cf8bc622a81a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>extralen</name>
      <anchorfile>structusb__interface__descriptor.html</anchorfile>
      <anchor>ac0a7dff17ffc27fbab51c6c3da13bc8b</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_element_descriptor</name>
    <filename>structusb__midi__element__descriptor.html</filename>
    <member kind="variable">
      <type>struct usb_midi_element_descriptor_head</type>
      <name>head</name>
      <anchorfile>structusb__midi__element__descriptor.html</anchorfile>
      <anchor>aceb705a015eba7b502de40d850c91eb4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct usb_midi_element_descriptor_body</type>
      <name>source</name>
      <anchorfile>structusb__midi__element__descriptor.html</anchorfile>
      <anchor>aeee5e8ae77bf5670b1069b7ae9532cba</anchor>
      <arglist>[1]</arglist>
    </member>
    <member kind="variable">
      <type>struct usb_midi_element_descriptor_tail</type>
      <name>tail</name>
      <anchorfile>structusb__midi__element__descriptor.html</anchorfile>
      <anchor>a306ed382a9fee059245496738b48e740</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_element_descriptor_body</name>
    <filename>structusb__midi__element__descriptor__body.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>baSourceID</name>
      <anchorfile>structusb__midi__element__descriptor__body.html</anchorfile>
      <anchor>a56ccae27bd2185a3a49d3eef491dd562</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>baSourcePin</name>
      <anchorfile>structusb__midi__element__descriptor__body.html</anchorfile>
      <anchor>a9b77f722793a39050896084c13c43b6d</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_element_descriptor_head</name>
    <filename>structusb__midi__element__descriptor__head.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__midi__element__descriptor__head.html</anchorfile>
      <anchor>a1613513e34783e2dfc39611d9379b4ef</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__midi__element__descriptor__head.html</anchorfile>
      <anchor>a9db38eb848c29bbe685b5e45f86aa9fa</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>structusb__midi__element__descriptor__head.html</anchorfile>
      <anchor>a1173d32b63f2677677a722af6e5031bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bElementID</name>
      <anchorfile>structusb__midi__element__descriptor__head.html</anchorfile>
      <anchor>af9a621981df317f25f0d629e3045c12a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNrInputPins</name>
      <anchorfile>structusb__midi__element__descriptor__head.html</anchorfile>
      <anchor>af74cec8e345b84098e63feeb07ee2479</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_element_descriptor_tail</name>
    <filename>structusb__midi__element__descriptor__tail.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNrOutputPins</name>
      <anchorfile>structusb__midi__element__descriptor__tail.html</anchorfile>
      <anchor>a58b3db8faa40dbd42db4ff0a351de91d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bInTerminalLink</name>
      <anchorfile>structusb__midi__element__descriptor__tail.html</anchorfile>
      <anchor>ac76d2803aa85e9b2b09f7995fccc13f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bOutTerminalLink</name>
      <anchorfile>structusb__midi__element__descriptor__tail.html</anchorfile>
      <anchor>a264924a4abb2c6e27eeffa13c6bd598b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bElCapsSize</name>
      <anchorfile>structusb__midi__element__descriptor__tail.html</anchorfile>
      <anchor>aba5ed4471ad63dd812b17df04a3e1daa</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bmElementCaps</name>
      <anchorfile>structusb__midi__element__descriptor__tail.html</anchorfile>
      <anchor>a53c8895e3929d39d4353ff8d80ae26e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iElement</name>
      <anchorfile>structusb__midi__element__descriptor__tail.html</anchorfile>
      <anchor>ace2fcaef278792de3ee16706818cd957</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_endpoint_descriptor</name>
    <filename>structusb__midi__endpoint__descriptor.html</filename>
    <member kind="variable">
      <type>struct usb_midi_endpoint_descriptor_head</type>
      <name>head</name>
      <anchorfile>structusb__midi__endpoint__descriptor.html</anchorfile>
      <anchor>a256a365e984c9f6f80586755b2750f04</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct usb_midi_endpoint_descriptor_body</type>
      <name>jack</name>
      <anchorfile>structusb__midi__endpoint__descriptor.html</anchorfile>
      <anchor>a78626cc5657e05998b55fc3d346a7437</anchor>
      <arglist>[1]</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_endpoint_descriptor_body</name>
    <filename>structusb__midi__endpoint__descriptor__body.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>baAssocJackID</name>
      <anchorfile>structusb__midi__endpoint__descriptor__body.html</anchorfile>
      <anchor>a3c4231557c270c5c9c919c5f13d066ab</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_endpoint_descriptor_head</name>
    <filename>structusb__midi__endpoint__descriptor__head.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__midi__endpoint__descriptor__head.html</anchorfile>
      <anchor>acf79b103e6796513067e4ffb838c99db</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__midi__endpoint__descriptor__head.html</anchorfile>
      <anchor>a9fade3298e6b9ee37df3e0159ad02a8e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubType</name>
      <anchorfile>structusb__midi__endpoint__descriptor__head.html</anchorfile>
      <anchor>ab80ffc4c73740580c3e75105b10abd1c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNumEmbMIDIJack</name>
      <anchorfile>structusb__midi__endpoint__descriptor__head.html</anchorfile>
      <anchor>a1b7f4b897e19a2a19829a3a45a8776fb</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_header_descriptor</name>
    <filename>structusb__midi__header__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__midi__header__descriptor.html</anchorfile>
      <anchor>a71087f6c01aaf933a8186974dc4b98a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__midi__header__descriptor.html</anchorfile>
      <anchor>a38677aa3a7ae341c58b8d1f6296ce1f0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>structusb__midi__header__descriptor.html</anchorfile>
      <anchor>acf52a9886f0c38b96316b8b9e4abc84c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>bcdMSC</name>
      <anchorfile>structusb__midi__header__descriptor.html</anchorfile>
      <anchor>a11940f34334b36ef8a86aa3a660aa93b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wTotalLength</name>
      <anchorfile>structusb__midi__header__descriptor.html</anchorfile>
      <anchor>afbce7cbe6f6153b4438abf653fe81fdd</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_in_jack_descriptor</name>
    <filename>structusb__midi__in__jack__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__midi__in__jack__descriptor.html</anchorfile>
      <anchor>a229fba7c0a7de06c1d4638a931235d28</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__midi__in__jack__descriptor.html</anchorfile>
      <anchor>a5837aff52eaf6ba4d46162e224a4c343</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>structusb__midi__in__jack__descriptor.html</anchorfile>
      <anchor>ab4da4280a831944a77a096e19b650525</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bJackType</name>
      <anchorfile>structusb__midi__in__jack__descriptor.html</anchorfile>
      <anchor>aed3ddc3a4cc4f411629f6f3e3861a398</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bJackID</name>
      <anchorfile>structusb__midi__in__jack__descriptor.html</anchorfile>
      <anchor>af0d46ecb58995a0db0162102fbce62b9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iJack</name>
      <anchorfile>structusb__midi__in__jack__descriptor.html</anchorfile>
      <anchor>a2ce84280ede3290b9070342f6b1bdc14</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_out_jack_descriptor</name>
    <filename>structusb__midi__out__jack__descriptor.html</filename>
    <member kind="variable">
      <type>struct usb_midi_out_jack_descriptor_head</type>
      <name>head</name>
      <anchorfile>structusb__midi__out__jack__descriptor.html</anchorfile>
      <anchor>ac2b0e4425cf1122ffc97ee49cc7016c1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct usb_midi_out_jack_descriptor_body</type>
      <name>source</name>
      <anchorfile>structusb__midi__out__jack__descriptor.html</anchorfile>
      <anchor>af7b76b530ba28038a6f56a2bf9a6b5fd</anchor>
      <arglist>[1]</arglist>
    </member>
    <member kind="variable">
      <type>struct usb_midi_out_jack_descriptor_tail</type>
      <name>tail</name>
      <anchorfile>structusb__midi__out__jack__descriptor.html</anchorfile>
      <anchor>ad70077e82ee963308027c20645cb3a90</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_out_jack_descriptor_body</name>
    <filename>structusb__midi__out__jack__descriptor__body.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>baSourceID</name>
      <anchorfile>structusb__midi__out__jack__descriptor__body.html</anchorfile>
      <anchor>aa1ebef6fbc800a24bd32a37e6022d81e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>baSourcePin</name>
      <anchorfile>structusb__midi__out__jack__descriptor__body.html</anchorfile>
      <anchor>a8647e473dfeacbf8ccc03bbada796802</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_out_jack_descriptor_head</name>
    <filename>structusb__midi__out__jack__descriptor__head.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__midi__out__jack__descriptor__head.html</anchorfile>
      <anchor>a1e946d2f405352b9b309459d7aea3993</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__midi__out__jack__descriptor__head.html</anchorfile>
      <anchor>a20c92b4ba4bdbbed5fb44cc9139fa01a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorSubtype</name>
      <anchorfile>structusb__midi__out__jack__descriptor__head.html</anchorfile>
      <anchor>a5516979e528f5e0984576d5cb75646f3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bJackType</name>
      <anchorfile>structusb__midi__out__jack__descriptor__head.html</anchorfile>
      <anchor>ab06ae4e8630351229a6ecc01aedaf460</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bJackID</name>
      <anchorfile>structusb__midi__out__jack__descriptor__head.html</anchorfile>
      <anchor>ac208961c0f43a2c9bb6da46f00f09db6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bNrInputPins</name>
      <anchorfile>structusb__midi__out__jack__descriptor__head.html</anchorfile>
      <anchor>a7150189cb6f34f6662304c5648b38916</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_midi_out_jack_descriptor_tail</name>
    <filename>structusb__midi__out__jack__descriptor__tail.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>iJack</name>
      <anchorfile>structusb__midi__out__jack__descriptor__tail.html</anchorfile>
      <anchor>a63d1b9c91210db97e930345a5d268616</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_msc_cbw</name>
    <filename>structusb__msc__cbw.html</filename>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCBWSignature</name>
      <anchorfile>structusb__msc__cbw.html</anchorfile>
      <anchor>a06c9f47162709307ab5f5b68c96a3119</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCBWTag</name>
      <anchorfile>structusb__msc__cbw.html</anchorfile>
      <anchor>a3b3a4d72d95d8ed9fdfb66d06250e546</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCBWDataTransferLength</name>
      <anchorfile>structusb__msc__cbw.html</anchorfile>
      <anchor>af2ae2bce99068e5b4ea7081f023bab2a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmCBWFlags</name>
      <anchorfile>structusb__msc__cbw.html</anchorfile>
      <anchor>a8331a5d04e78012779cdf5a568af90b9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bCBWLUN</name>
      <anchorfile>structusb__msc__cbw.html</anchorfile>
      <anchor>a10cb317c0fe88d24265168592ed63c6b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bCBWCBLength</name>
      <anchorfile>structusb__msc__cbw.html</anchorfile>
      <anchor>aa27e149aae7423c6dc3e430cb5a57795</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>CBWCB</name>
      <anchorfile>structusb__msc__cbw.html</anchorfile>
      <anchor>a9b19b64641e14f1f74bc51b0af55974f</anchor>
      <arglist>[16]</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_msc_csw</name>
    <filename>structusb__msc__csw.html</filename>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCSWSignature</name>
      <anchorfile>structusb__msc__csw.html</anchorfile>
      <anchor>aebc2d944ccc8284dbd9e836d30089aad</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCSWTag</name>
      <anchorfile>structusb__msc__csw.html</anchorfile>
      <anchor>a4c8ecc71a403a7050f071b964028cd73</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>dCSWDataResidue</name>
      <anchorfile>structusb__msc__csw.html</anchorfile>
      <anchor>af6a50ce0fb8b6daa4dc40167daffa83e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bCSWStatus</name>
      <anchorfile>structusb__msc__csw.html</anchorfile>
      <anchor>aa1c2d8d8b2b16b50e9d57e60fd19ca2f</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_msc_trans</name>
    <filename>structusb__msc__trans.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>cbw_cnt</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a2a19069abdeca0556cbe90c8af463fde</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct usb_msc_cbw</type>
      <name>cbw</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>af1b071cf466021ddc18cae7994c069f7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>buf</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>aabde15222ad83f6621f744350ffc51ca</anchor>
      <arglist>[1]</arglist>
    </member>
    <member kind="variable">
      <type>union usb_msc_trans::@0</type>
      <name>cbw</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a1d117026e81d2374395c64d7bc76bd7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>bytes_to_read</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a3ab127622f76bed43aecf520efb67e24</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>bytes_to_write</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a46e339b0ba4696a2f252b3b7e14379dc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>byte_count</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a7e1db671852cfcf6f515136a8f76fcbb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>lba_start</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a5fc80ac1c9a0eb75995e534c1d308df7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>block_count</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a7d3252979406f9de573fba4907c8a1b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>current_block</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a20a39235ec981a6fa29e9fc739d9b4a9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>msd_buf</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a4725dac7dd085810edbf6962090b108c</anchor>
      <arglist>[512]</arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>csw_valid</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>adbb4ba8f0aa484d37e01e98a504448d6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>csw_sent</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a98d0d138ab010bba07b00744ec2b248a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>struct usb_msc_csw</type>
      <name>csw</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>a43600580255023408f08061721826938</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>union usb_msc_trans::@1</type>
      <name>csw</name>
      <anchorfile>structusb__msc__trans.html</anchorfile>
      <anchor>aab66425ce97f007e56265b3c9f6835bc</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_setup_data</name>
    <filename>structusb__setup__data.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bmRequestType</name>
      <anchorfile>structusb__setup__data.html</anchorfile>
      <anchor>a593d22b430d1bd2f80a7c8795a4659fb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bRequest</name>
      <anchorfile>structusb__setup__data.html</anchorfile>
      <anchor>a30dbf7050fed7686bdf9a87c2dd1eb5c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wValue</name>
      <anchorfile>structusb__setup__data.html</anchorfile>
      <anchor>a8c0e8f7a6d919a7a05c93f6cf5b22b0b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wIndex</name>
      <anchorfile>structusb__setup__data.html</anchorfile>
      <anchor>a2ed7d2d64541da9f237a13435edd3f3a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wLength</name>
      <anchorfile>structusb__setup__data.html</anchorfile>
      <anchor>ae8dc4ecb24d13bb2ad31ab8b313fd3e2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>usb_string_descriptor</name>
    <filename>structusb__string__descriptor.html</filename>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bLength</name>
      <anchorfile>structusb__string__descriptor.html</anchorfile>
      <anchor>a93d9bb7c2c44f6f0cae1a871a4a18789</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>bDescriptorType</name>
      <anchorfile>structusb__string__descriptor.html</anchorfile>
      <anchor>a37d44e07cb6d5b449b03fb70c9677b15</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>wData</name>
      <anchorfile>structusb__string__descriptor.html</anchorfile>
      <anchor>a7a4591429d5e3f3ecc574b31596f851b</anchor>
      <arglist>[]</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>_usbd_device::user_control_callback</name>
    <filename>struct__usbd__device_1_1user__control__callback.html</filename>
    <member kind="variable">
      <type>usbd_control_callback</type>
      <name>cb</name>
      <anchorfile>struct__usbd__device_1_1user__control__callback.html</anchorfile>
      <anchor>a55fd060a0418feea229f2b3b05e5c808</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>type</name>
      <anchorfile>struct__usbd__device_1_1user__control__callback.html</anchorfile>
      <anchor>abc76a06a989cd6da83b48edbc5e864b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint8_t</type>
      <name>type_mask</name>
      <anchorfile>struct__usbd__device_1_1user__control__callback.html</anchorfile>
      <anchor>ade3ae61e072295839d83cd17c6aa804a</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_audio_defines</name>
    <title>USB Audio Type Definitions</title>
    <filename>group__usb__audio__defines.html</filename>
    <class kind="struct">usb_audio_header_descriptor_head</class>
    <class kind="struct">usb_audio_header_descriptor_body</class>
    <class kind="struct">usb_midi_header_descriptor</class>
    <class kind="struct">usb_midi_in_jack_descriptor</class>
    <class kind="struct">usb_midi_out_jack_descriptor_head</class>
    <class kind="struct">usb_midi_out_jack_descriptor_body</class>
    <class kind="struct">usb_midi_out_jack_descriptor_tail</class>
    <class kind="struct">usb_midi_out_jack_descriptor</class>
    <class kind="struct">usb_midi_element_descriptor_head</class>
    <class kind="struct">usb_midi_element_descriptor_body</class>
    <class kind="struct">usb_midi_element_descriptor_tail</class>
    <class kind="struct">usb_midi_element_descriptor</class>
    <class kind="struct">usb_midi_endpoint_descriptor_head</class>
    <class kind="struct">usb_midi_endpoint_descriptor_body</class>
    <class kind="struct">usb_midi_endpoint_descriptor</class>
    <member kind="function">
      <type>struct usb_audio_header_descriptor_head</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__audio__defines.html</anchorfile>
      <anchor>gaaee0a5ff0aa053fd97f61a97733cac25</anchor>
      <arglist>((packed))</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_cdc_defines</name>
    <title>USB CDC Type Definitions</title>
    <filename>group__usb__cdc__defines.html</filename>
    <class kind="struct">usb_cdc_header_descriptor</class>
    <class kind="struct">usb_cdc_union_descriptor</class>
    <class kind="struct">usb_cdc_call_management_descriptor</class>
    <class kind="struct">usb_cdc_acm_descriptor</class>
    <class kind="struct">usb_cdc_line_coding</class>
    <class kind="struct">usb_cdc_notification</class>
    <member kind="enumeration">
      <type></type>
      <name>usb_cdc_line_coding_bCharFormat</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>ga7edb29528e9dcced43aaca511dbd9bc2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_1_STOP_BITS</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga7edb29528e9dcced43aaca511dbd9bc2a20790d5ae5573b13e38a5848ed4f8437</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_1_5_STOP_BITS</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga7edb29528e9dcced43aaca511dbd9bc2a528bc870457b07f3e1df4f54b89aef77</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_2_STOP_BITS</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga7edb29528e9dcced43aaca511dbd9bc2a88105b55058eb7b4e1a22f0e442801b9</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>usb_cdc_line_coding_bParityType</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>ga0cabe0e00dd7200ff5e90f0119fbf66f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_NO_PARITY</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga0cabe0e00dd7200ff5e90f0119fbf66fa84dcd5abce5bd2c699749b70511bf95a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_ODD_PARITY</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga0cabe0e00dd7200ff5e90f0119fbf66fa2c2ea42ec25ac66d613f57416fa4b35d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_EVEN_PARITY</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga0cabe0e00dd7200ff5e90f0119fbf66fa1a69931078ca04ed352f2a850f42ba63</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_MARK_PARITY</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga0cabe0e00dd7200ff5e90f0119fbf66fa3ce606e8526fd03fef527027bb28b160</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_CDC_SPACE_PARITY</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gga0cabe0e00dd7200ff5e90f0119fbf66fa4ff36fa9da45d9cbcbc10638e23a32dc</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>struct usb_cdc_header_descriptor</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>gab6677e82689abb017cf4e5590b1305fa</anchor>
      <arglist>((packed))</arglist>
    </member>
    <member kind="variable">
      <type>enum usb_cdc_line_coding_bCharFormat</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__cdc__defines.html</anchorfile>
      <anchor>ga8cb92f50fd0c1ad2ab1195289f72bff6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_dfu_defines</name>
    <title>USB DFU Type Definitions</title>
    <filename>group__usb__dfu__defines.html</filename>
    <class kind="struct">usb_dfu_descriptor</class>
    <member kind="enumeration">
      <type></type>
      <name>dfu_req</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ga1eb1afd96bccc953e4c19022df7d2171</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_DETACH</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171ab8e18a7fd7b5b1899c5b57fa69faac69</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_DNLOAD</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171aa3d14f787e4880e7c6b931f05070330f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_UPLOAD</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171af7427780a3977ec43a36b102c639289c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_GETSTATUS</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171a29e1b58118f9b9e096b293b365bdc91b</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_CLRSTATUS</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171a8adc62cb245219266a0a567053633a1e</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_GETSTATE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171a07c028e3b420dac6bf986ffbe971d2f0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_ABORT</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga1eb1afd96bccc953e4c19022df7d2171ac4cdec2bd3bc05311e89fc989ca0c61d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>dfu_status</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gabcf2757cf1c6281a06a8a6f25ff2aa31</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_OK</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a3a550674419538042b365d365595dc40</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_TARGET</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a9a7a7639b542fa571c5bf230e811afa5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_FILE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a507bb0dd505ecf8e1ad31f3ad59ca595</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_WRITE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a714bc30afe28582ddb38e90702beb1d3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_ERASE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a69f5911f3539cf83984e2220e069d9d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_CHECK_ERASED</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a1b79f547824d24336bd32c4facf90e35</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_PROG</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a334d3e00203a36c177a36c4f20ca4cd3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_VERIFY</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31ab209da0e18d2bdac90cce75446f5e284</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_ADDRESS</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31ab7abfa9c48bb721cdbc63a220e0a34a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_NOTDONE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a9f0b8e1bdb57da25a84e499db614fa38</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_FIRMWARE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31aaebd82c354d409a6a918f103bbfe6c26</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_VENDOR</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31ad5c270162ee2af751292540632a538c2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_USBR</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31abc69336005f3df189e7df6d0f0ad69b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_POR</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a4b8d062347f43c1d75a58ed4e74d63bb</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_UNKNOWN</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31afeee9594f62632238db9053c37a99465</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>DFU_STATUS_ERR_STALLEDPKT</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ggabcf2757cf1c6281a06a8a6f25ff2aa31a8d963dfec7ec03c538c5a38944f63fe3</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>dfu_state</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ga3f025bde53c777c5e0493ef54bbeb143</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_APP_IDLE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a2071f71d4f5fb2435f161d1a70167ec1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_APP_DETACH</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a1cae6396e56ba1edfd3bc7bf4b9b6dd4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_IDLE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a72332acd446920c416a5b4da079b33d5</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_DNLOAD_SYNC</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a2c74cfc90fa6f9e9df8bce3fca9d3ed4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_DNBUSY</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a262a372a18004a540f3486c9d6eb77de</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_DNLOAD_IDLE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a5658b351cf06fcd7af8f11f911444dab</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_MANIFEST_SYNC</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a779788d8fec7d6397b6a352ce62940b7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_MANIFEST</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143acd82927685281eb302cf3b33330cb4ac</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_MANIFEST_WAIT_RESET</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a15938a3a729a93ff2003e7c247be8371</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_UPLOAD_IDLE</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143a86181ad59e744acb655eaa940cdf42d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>STATE_DFU_ERROR</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gga3f025bde53c777c5e0493ef54bbeb143ac2c6d30505ea22bcc31222a41cb4815b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>struct usb_dfu_descriptor</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>ga53297a7dd343d62a8a8efde4909b3248</anchor>
      <arglist>((packed))</arglist>
    </member>
    <member kind="variable">
      <type>enum dfu_req</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__dfu__defines.html</anchorfile>
      <anchor>gafab3ce003dc4e9015c96f17748178a0f</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>USB</name>
    <title>Generic USB</title>
    <filename>group__USB.html</filename>
    <subgroup>usb_drivers_file</subgroup>
    <subgroup>usb_control_file</subgroup>
    <subgroup>usb_standard_file</subgroup>
  </compound>
  <compound kind="group">
    <name>USB_defines</name>
    <title>Generic USB Defines</title>
    <filename>group__USB__defines.html</filename>
    <subgroup>usb_audio_defines</subgroup>
    <subgroup>usb_cdc_defines</subgroup>
    <subgroup>usb_dfu_defines</subgroup>
    <subgroup>usb_hid_defines</subgroup>
    <subgroup>usb_msc_defines</subgroup>
    <subgroup>usb_driver_defines</subgroup>
    <subgroup>usb_type_defines</subgroup>
    <subgroup>usb_private_defines</subgroup>
  </compound>
  <compound kind="group">
    <name>usb_hid_defines</name>
    <title>USB HID Type Definitions</title>
    <filename>group__usb__hid__defines.html</filename>
    <class kind="struct">usb_hid_descriptor</class>
    <member kind="function">
      <type>struct usb_hid_descriptor</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__hid__defines.html</anchorfile>
      <anchor>gab06170e95e244e47f1d5c11ced1a483f</anchor>
      <arglist>((packed))</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_msc_defines</name>
    <title>USB MSC Type Definitions</title>
    <filename>group__usb__msc__defines.html</filename>
    <member kind="typedef">
      <type>struct _usbd_mass_storage</type>
      <name>usbd_mass_storage</name>
      <anchorfile>group__usb__msc__defines.html</anchorfile>
      <anchor>ga38f5f9afaaae0a35c540c5d2d59dfd57</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>usbd_mass_storage *</type>
      <name>usb_msc_init</name>
      <anchorfile>group__usb__msc__defines.html</anchorfile>
      <anchor>ga5e6959c3ac6ff4efab4fd3b59353f497</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ep_in, uint8_t ep_in_size, uint8_t ep_out, uint8_t ep_out_size, const char *vendor_id, const char *product_id, const char *product_revision_level, const uint32_t block_count, int(*read_block)(uint32_t lba, uint8_t *copy_to), int(*write_block)(uint32_t lba, const uint8_t *copy_from))</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_driver_defines</name>
    <title>USB Drivers</title>
    <filename>group__usb__driver__defines.html</filename>
    <member kind="typedef">
      <type>struct _usbd_driver</type>
      <name>usbd_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaf89d061a4e982ccc705dbf1fc7f14fca</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>struct _usbd_device</type>
      <name>usbd_device</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga398cbe724daaf03b101d88163e7c1fce</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>int(*</type>
      <name>usbd_control_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaf05abc6fde0ddaf3b630222a8422579f</anchor>
      <arglist>)(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void(**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))</arglist>
    </member>
    <member kind="enumeration">
      <type></type>
      <name>usbd_request_return_codes</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gae09dc4bce47d0848ddacbd8b08ef76a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USBD_REQ_NOTSUPP</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ggae09dc4bce47d0848ddacbd8b08ef76a7ae6fb6de05148b303504d1f2855534495</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USBD_REQ_HANDLED</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ggae09dc4bce47d0848ddacbd8b08ef76a7ae44b9555863f8ecd5415bef01cb5779f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USBD_REQ_NEXT_CALLBACK</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ggae09dc4bce47d0848ddacbd8b08ef76a7a2de82a2c4708a6cd78384a3362e86d7f</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>usbd_device *</type>
      <name>usbd_init</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga8bbc488106ca72c2b550c1c9f89d5748</anchor>
      <arglist>(const usbd_driver *driver, const struct usb_device_descriptor *dev, const struct usb_config_descriptor *conf, const char **strings, int num_strings, uint8_t *control_buffer, uint16_t control_buffer_size)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_reset_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga2cdd6f2cd24e7e222b2add6e08283a7e</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_suspend_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga064f9d058427b1fb6f40b35e03a8cd5b</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_resume_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga004c77a73e4dcd6216678925f3fa1f41</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_sof_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gad8e688f14cac90c4933954e3ebbfc3b7</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>usbd_register_control_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga86f80b7593ee05d364fd791896213e38</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t type, uint8_t type_mask, usbd_control_callback callback)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>usbd_register_set_config_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gab61ace3fbfd23ca3bc3afcee6c231d05</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(usbd_device *usbd_dev, uint16_t wValue))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_set_altsetting_callback</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga21d5f43101d1e56baf0d9f664401e506</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_poll</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga94eba5aff7befd8009cea2f2abb43134</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_disconnect</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaef20aadd330e4bb9e97635eeb751d8f9</anchor>
      <arglist>(usbd_device *usbd_dev, bool disconnected)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_setup</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga888110ac1598406bb834b494c8db7f6c</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t type, uint16_t max_size, void(*callback)(usbd_device *usbd_dev, uint8_t ep))</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>usbd_ep_write_packet</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaa5b245bdd9757cdc79d1fe721ebcf03f</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, const void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>usbd_ep_read_packet</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gabd42dc86361dbf9c639fd883ada1f572</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_stall_set</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga6d16be856512df298e40dfa31a216172</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t stall)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>usbd_ep_stall_get</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga396b34ccbe5bf6718417ef3ee0c3b788</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_nak_set</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga2f2a7dac5031fd8d1b5eb076d3f24b42</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t nak)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_cable_connect</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga3c83b9f71370e276c4550b25f013d833</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t on)</arglist>
    </member>
    <member kind="variable">
      <type>BEGIN_DECLS enum usbd_request_return_codes</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga2530c9cbb5ab3be81daa4e988c2ce579</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const usbd_driver</type>
      <name>stm32f103_usb_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>gaf75394477d6005c2032c0d2aebb7a9a6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const usbd_driver</type>
      <name>stm32f107_usb_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga5adac4ccb88c72946fa692670d0bfc16</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const usbd_driver</type>
      <name>stm32f207_usb_driver</name>
      <anchorfile>group__usb__driver__defines.html</anchorfile>
      <anchor>ga2170186d661d2005c6b18c06d7f49f12</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_type_defines</name>
    <title>USB Standard Structure Definitions</title>
    <filename>group__usb__type__defines.html</filename>
    <class kind="struct">usb_setup_data</class>
    <class kind="struct">usb_device_descriptor</class>
    <class kind="struct">usb_device_qualifier_descriptor</class>
    <class kind="struct">usb_config_descriptor</class>
    <class kind="struct">usb_interface_descriptor</class>
    <class kind="struct">usb_endpoint_descriptor</class>
    <class kind="struct">usb_string_descriptor</class>
    <class kind="struct">usb_iface_assoc_descriptor</class>
    <member kind="enumeration">
      <type></type>
      <name>usb_language_id</name>
      <anchorfile>group__usb__type__defines.html</anchorfile>
      <anchor>gaca86007d66ae94421fd4f93c0b96b04a</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_LANGID_ENGLISH_US</name>
      <anchorfile>group__usb__type__defines.html</anchorfile>
      <anchor>ggaca86007d66ae94421fd4f93c0b96b04aa069b825fab4882505aca430300ecc9a8</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>struct usb_setup_data</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__type__defines.html</anchorfile>
      <anchor>gaf45ea7b5ea10239fbbd9377f7faae445</anchor>
      <arglist>((packed))</arglist>
    </member>
    <member kind="variable">
      <type>enum usb_language_id</type>
      <name>__attribute__</name>
      <anchorfile>group__usb__type__defines.html</anchorfile>
      <anchor>ga993287986228cc24f80bfdf807896554</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_drivers_file</name>
    <title>Generic USB Drivers</title>
    <filename>group__usb__drivers__file.html</filename>
    <member kind="function">
      <type>usbd_device *</type>
      <name>usbd_init</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga8bbc488106ca72c2b550c1c9f89d5748</anchor>
      <arglist>(const usbd_driver *driver, const struct usb_device_descriptor *dev, const struct usb_config_descriptor *conf, const char **strings, int num_strings, uint8_t *control_buffer, uint16_t control_buffer_size)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_reset_callback</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga2cdd6f2cd24e7e222b2add6e08283a7e</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_suspend_callback</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga064f9d058427b1fb6f40b35e03a8cd5b</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_resume_callback</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga004c77a73e4dcd6216678925f3fa1f41</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_sof_callback</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>gad8e688f14cac90c4933954e3ebbfc3b7</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(void))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>_usbd_reset</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga3bc8dd33500621a1e900c78a42dcaf27</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_poll</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga94eba5aff7befd8009cea2f2abb43134</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_disconnect</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>gaef20aadd330e4bb9e97635eeb751d8f9</anchor>
      <arglist>(usbd_device *usbd_dev, bool disconnected)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_setup</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga888110ac1598406bb834b494c8db7f6c</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t type, uint16_t max_size, void(*callback)(usbd_device *usbd_dev, uint8_t ep))</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>usbd_ep_write_packet</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>gaa5b245bdd9757cdc79d1fe721ebcf03f</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, const void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>usbd_ep_read_packet</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>gabd42dc86361dbf9c639fd883ada1f572</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, void *buf, uint16_t len)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_stall_set</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga6d16be856512df298e40dfa31a216172</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t stall)</arglist>
    </member>
    <member kind="function">
      <type>uint8_t</type>
      <name>usbd_ep_stall_get</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga396b34ccbe5bf6718417ef3ee0c3b788</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_ep_nak_set</name>
      <anchorfile>group__usb__drivers__file.html</anchorfile>
      <anchor>ga2f2a7dac5031fd8d1b5eb076d3f24b42</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t addr, uint8_t nak)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_control_file</name>
    <title>Generic USB Control Requests</title>
    <filename>group__usb__control__file.html</filename>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>stall_transaction</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>gaf6f522923d66ffdea79fbbd4decc31d4</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>usbd_register_control_callback</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>ga86f80b7593ee05d364fd791896213e38</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t type, uint8_t type_mask, usbd_control_callback callback)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>usb_control_send_chunk</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>ga46375610ee21a2c8ab4a06fe3bd70291</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_control_recv_chunk</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>gaf3b08c6a689c9f5a20f636507e70bb52</anchor>
      <arglist>(usbd_device *usbd_dev)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_control_request_dispatch</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>ga883123627c0bed716ec41d4d7f3c0a46</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>usb_control_setup_read</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>ga2149cd30a26a18c9c46e32db4aa5ef3e</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>usb_control_setup_write</name>
      <anchorfile>group__usb__control__file.html</anchorfile>
      <anchor>gab9305438ecc3aaa7359aacdbc4f55c29</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_private_defines</name>
    <title>USB Private Structures</title>
    <filename>group__usb__private__defines.html</filename>
    <class kind="struct">_usbd_device</class>
    <member kind="enumeration">
      <type></type>
      <name>_usbd_transaction</name>
      <anchorfile>group__usb__private__defines.html</anchorfile>
      <anchor>ga1cf959928954ac5fe754b82f75ce4994</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_TRANSACTION_IN</name>
      <anchorfile>group__usb__private__defines.html</anchorfile>
      <anchor>gga1cf959928954ac5fe754b82f75ce4994a9d186519ca162685e9cb4d3c17b8fdff</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_TRANSACTION_OUT</name>
      <anchorfile>group__usb__private__defines.html</anchorfile>
      <anchor>gga1cf959928954ac5fe754b82f75ce4994ab016c2caf10af8d5cd6adbb926a18c90</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>USB_TRANSACTION_SETUP</name>
      <anchorfile>group__usb__private__defines.html</anchorfile>
      <anchor>gga1cf959928954ac5fe754b82f75ce4994a7439308c92f9f23a0e750befc7f94018</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_standard_file</name>
    <title>Generic USB Standard Request Interface</title>
    <filename>group__usb__standard__file.html</filename>
    <member kind="function">
      <type>int</type>
      <name>usbd_register_set_config_callback</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>gab61ace3fbfd23ca3bc3afcee6c231d05</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(usbd_device *usbd_dev, uint16_t wValue))</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>usbd_register_set_altsetting_callback</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga21d5f43101d1e56baf0d9f664401e506</anchor>
      <arglist>(usbd_device *usbd_dev, void(*callback)(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue))</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static uint16_t</type>
      <name>build_config_descriptor</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>gaf8a21c66ec88535a258872791c468098</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t index, uint8_t *buf, uint16_t len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_descriptor_type</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga2df65bb0dcba3643cde420b3fdc08ff0</anchor>
      <arglist>(uint16_t wValue)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_descriptor_index</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>gaf52328d55b34702e49cb14ec9005ca19</anchor>
      <arglist>(uint16_t wValue)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_get_descriptor</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>gad9327961f96b6203ac232d69d53afcf6</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_set_address</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga2e39a66a58cc636b01e3d90cb70c7625</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_set_configuration</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga11f2ca9802883c0ff1dc312799e7250d</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_get_configuration</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga430326cdf658101dd4b02d52ebb0cf3c</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_set_interface</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga137aae06450b52a0c22f7ffa6fba5ba0</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_get_interface</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga22c09fa8b89c6d0cb46e290cfecdf33f</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_device_get_status</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga2c512d44bd72a83d9b438859ab3ee352</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_interface_get_status</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga3af045144aef2572263f2ec0a2fdbef9</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_endpoint_get_status</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga4cfdd2484f7002da2cc98eea1e6a69e9</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_endpoint_stall</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>gad24d6daf25ffd5539979b9ea1648ec6f</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static int</type>
      <name>usb_standard_endpoint_unstall</name>
      <anchorfile>group__usb__standard__file.html</anchorfile>
      <anchor>ga9714d02e4367d72d66f7d71abe10a70f</anchor>
      <arglist>(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len)</arglist>
    </member>
  </compound>
  <compound kind="group">
    <name>usb_msc</name>
    <title>Usb_msc</title>
    <filename>group__usb__msc.html</filename>
    <member kind="function">
      <type>usbd_mass_storage *</type>
      <name>usb_msc_init</name>
      <anchorfile>group__usb__msc.html</anchorfile>
      <anchor>ga5e6959c3ac6ff4efab4fd3b59353f497</anchor>
      <arglist>(usbd_device *usbd_dev, uint8_t ep_in, uint8_t ep_in_size, uint8_t ep_out, uint8_t ep_out_size, const char *vendor_id, const char *product_id, const char *product_revision_level, const uint32_t block_count, int(*read_block)(uint32_t lba, uint8_t *copy_to), int(*write_block)(uint32_t lba, const uint8_t *copy_from))</arglist>
    </member>
  </compound>
  <compound kind="page">
    <name>lgpl_license</name>
    <title>libopencm3 License</title>
    <filename>lgpl_license</filename>
  </compound>
  <compound kind="page">
    <name>index</name>
    <title>libopencm3 Generic USB</title>
    <filename>index</filename>
  </compound>
</tagfile>

#set page(width: 1200pt, height: 440pt, margin: (x: 28pt, y: 18pt), fill: white)
#set text(font: "SimSun", size: 12pt)

#let box-title(s) = text(size: 13pt, weight: "bold")[#s]
#let box-body(s) = text(size: 11pt)[#s]

#let block(title, body: none, width: auto) = box(
  width: width,
  inset: (x: 12pt, y: 10pt),
  radius: 4pt,
  stroke: (paint: luma(70), thickness: 1pt),
  fill: luma(245),
)[
  #stack(
    spacing: 6pt,
    box-title(title),
    if body != none { box-body(body) } else { none },
  )
]

#let group(title, body, width: auto) = box(
  width: width,
  inset: (x: 12pt, y: 10pt),
  radius: 4pt,
  stroke: (paint: luma(55), thickness: 1pt),
  fill: luma(252),
)[
  #stack(
    spacing: 8pt,
    box-title(title),
    body,
  )
]

#let tag(s) = box(
  inset: (x: 8pt, y: 5pt),
  radius: 3pt,
  stroke: (paint: luma(75), thickness: 0.8pt),
  fill: white,
)[
  #text(size: 11pt)[#s]
]

#let arrow-down() = text(size: 16pt, weight: "bold")[↓]

#let module-grid(items) = grid(
  columns: (1fr, 1fr, 1fr),
  gutter: 8pt,
  ..items.map(s => tag(s)),
)

#let main-col() = {
  let init = block(
    "入口与调度",
    body: [
      `main()` → `SYSCFG_DL_init()` → `App_init()` → `while(1) App_loop()`
    ],
  )

  let loop = group(
    "前台主循环（轮询任务链）",
    stack(
      spacing: 8pt,
      module-grid((
        "UserUART_task / processCommand",
        "serviceControls（按键/编码器/模式）",
        "servicePhaseLock → AdcCapture_start",
        "serviceADC → phaseLockProcess → DDS_update",
        "serviceCCDVisualLock → TSL1401_capture → applySingleOutput",
        "serviceFWatchdog / RGBLED_task",
      )),
      box(
        inset: (x: 10pt, y: 7pt),
        radius: 3pt,
        stroke: (paint: luma(80), thickness: 0.6pt),
        fill: luma(248),
      )[
        #text(size: 11pt, fill: luma(40))[
          两条闭环：ADC 相位锁（鉴相电压 → PI → POW/FTW），CCD 视觉锁（形状评分 → 相位/幅度微调）
        ]
      ],
    ),
  )

  let drivers = group(
    "驱动与板级功能（User/）",
    module-grid((
      "DDS.c / AD9959.c",
      "AdcCapture.c（ADC0/1+DMA）",
      "TSL1401.c（CCD采集/分析）",
      "DacOutput.c（DAC+DMA）",
      "UserUART.c",
      "BTN.c / Encoder.c / RGBLED.c / Tick.c",
    )),
  )

  stack(
    spacing: 10pt,
    init,
    align(center)[#arrow-down()],
    loop,
    align(center)[#arrow-down()],
    drivers,
  )
}

#let irq-col() = {
  group(
    "后台中断（IRQ）与回调",
    stack(
      spacing: 10pt,
      block("SysTick_Handler（1ms）", body: ["Tick_SysTickCallback()；并保活 DacOutput_service()"]),
      block("ADC0/ADC1_IRQHandler", body: ["AdcCapture_ADC0IRQ/ADC1IRQ：DMA完成→ready"]),
      block("GROUP1_IRQHandler", body: ["GROUP1_IRQCallback：编码器 A/B 相计数"]),
      box(
        inset: (x: 10pt, y: 7pt),
        radius: 3pt,
        stroke: (paint: luma(85), thickness: 0.6pt),
        fill: luma(252),
      )[
        #text(size: 11pt, fill: luma(40))[
          共享状态（ready/tick/encVal 等）由前台轮询读取，形成前后台协作
        ]
      ],
    ),
    width: 1fr,
  )
}

#grid(
  columns: (2.2fr, 1fr),
  gutter: 18pt,
  main-col(),
  irq-col(),
)

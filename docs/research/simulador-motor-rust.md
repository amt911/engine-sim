# Construye en Rust un motor que suena

La receta para un simulador de motor en tiempo real que genere su propio sonido ya está probada y es pública. La creó AngeTheGreat en **engine-sim** (C++, licencia MIT). Su núcleo es un modelo de gas **0D de volúmenes concentrados**, con flujo compresible por orificio entre cilindro, colectores y escape, una combustión de frente de llama y un sólido rígido 2D. Todo se integra a **10 kHz por defecto, con 8 subpasos para el fluido**. El sonido no sale de la física directamente: la presión en el colector de escape se sobremuestrea a 44,1 kHz y pasa por una cadena DSP con convolución por respuesta al impulso (IR). El siguiente salto lo marcan dos proyectos. Uno es el propio Ange, en su nuevo simulador de código cerrado. El otro es **ensim4** (C, MIT), que está abierto. Ambos sustituyen la línea de retardo del escape por un **solucionador 1D de las ecuaciones de Euler en volúmenes finitos**, que es el método de la industria (GT-POWER, WAVE, OpenWAM). Las lecturas clave se reparten así: para el gas, Toro (solucionadores de Riemann), Winterbone & Pearson, Blair y el grupo CMT de la Universitat Politècnica de València; para la termodinámica, Wiebe, Woschni y los polinomios NASA; para el sonido, el libro gratuito de guías de onda de Julius O. Smith y Baldan et al. (2015), además de la radiación de Levine–Schwinger y la convolución FFT particionada. La literatura de fluidos para gráficos (Stam, SPH) no sirve para el sonido, porque sus solucionadores son incompresibles y no propagan ondas acústicas. Para que corra en máquinas lentas cuentan más estas decisiones que la micro-optimización: una manivela analítica de 1 grado de libertad en lugar de un solucionador de restricciones, convolución FFT en lugar del FIR directo de 10.000 coeficientes que usa engine-sim, un búfer circular sin bloqueos hacia `cpal` y una calidad adaptativa gobernada por el llenado del búfer de audio. **No existe ningún port de engine-sim a Rust**, aunque el ecosistema de crates (cpal, rtrb, fundsp, fft-convolver, rubato) cubre todo lo necesario. Hay dos advertencias. Casi todos los artículos de motores (SAE) son de pago. Y ninguna fuente publica cifras de CPU de un solucionador 1D a frecuencia de audio, así que la viabilidad en máquinas antiguas hay que comprobarla midiendo.

## engine-sim funciona con 0D a 10 kHz y mucho DSP encima

### Tres bucles anidados y el audio como reloj maestro

engine-sim es una simulación de paso fijo con tres niveles. El paso físico es de **10 kHz por defecto** (`m_simulationFrequency = 10000`). Cada script de motor puede fijar el suyo, y los motores incluidos usan entre 5.000 y 40.000 Hz. El usuario puede cambiarlo en ejecución entre 400 Hz y 400 kHz con N + rueda del ratón ([simulator.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/simulator.cpp), [engine_sim_application.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/engine_sim_application.cpp)). Dentro de cada paso, el modelo de gas da **8 subpasos** (`m_fluidSimulationSteps = 8`), es decir, 80 kHz efectivos para el fluido ([piston_engine_simulator.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/piston_engine_simulator.cpp)). Eso explica los "80.000 fotogramas por segundo" que citó la prensa ([The Drive](https://www.thedrive.com/news/hear-this-absurdly-accurate-homebrew-engine-sim-rev-a-chevy-big-block)). El sólido rígido, en cambio, solo se resuelve a 10 kHz. En cada fotograma, el hilo principal calcula los pasos necesarios y los corrige con un controlador muy simple: si el búfer del sintetizador baja del objetivo de **100 ms**, usa `(pasos+1)·1,1`, y si lo supera, `(pasos−1)·0,9` ([simulator.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/simulator.cpp)). Un hilo aparte sobremuestrea y procesa el audio con un mutex y una variable de condición ([synthesizer.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/synthesizer.cpp)). Ninguna parte de la física corre a la frecuencia de audio. Este control por llenado del búfer es lo que mantiene el sonido continuo sin un hilo de tiempo real estricto, y conviene copiarlo, aunque sin mutex.

### El gas: volúmenes concentrados y flujo de orificio compresible

Cada volumen es un `GasSystem` con estos datos: moles, energía interna, volumen, un **vector de momento 2D** y una mezcla de combustible, O₂ e inerte. Es un gas ideal con 5 grados de libertad, así que **γ = 1,4 constante** ([gas_system.h](https://github.com/ange-yaghi/engine-sim/blob/master/include/gas_system.h)). Cada cilindro tiene tres volúmenes: colector de admisión, cilindro y colector de escape más primario. El gas fluye en cadena: plenum → runner → cilindro → escape → colector común → atmósfera. Cada enlace usa la **ecuación isentrópica de orificio con régimen bloqueado y no bloqueado**, calculada desde el lado de mayor presión ([gas_system.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/gas_system.cpp), [combustion_chamber.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/combustion_chamber.cpp)). Los coeficientes se calibran con datos de banco de flujo: la culata es una tabla de alzada → CFM a 28 inH₂O (`k_28inH2O`), y los carburadores usan 1,5 inHg (`k_carb`). La alzada sale de un perfil de leva muestreado, con el ángulo de leva igual a (cigüeñal + avance)/2 ([cylinder_head.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/cylinder_head.cpp), [camshaft.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/camshaft.cpp)). El aire de admisión llega **ya premezclado** a un dosado fijo, como en un carburador, y la mariposa sigue `k = cos(throttle·π/2)·k_entrada` ([intake.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/intake.cpp)).

Lo más importante para tu diseño es esto: **no hay ningún solucionador de ondas 1D**. La longitud del escape solo entra de cuatro formas: como volumen, como una **línea de retardo pura** (longitud/343 m/s), como un escalado de amplitud 1/L² y a través de la IR ([piston_engine_simulator.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/piston_engine_simulator.cpp), [delay_filter.h](https://github.com/ange-yaghi/engine-sim/blob/master/include/delay_filter.h)). El modelo reproduce bien la *temporización* de los pulsos (orden de encendido, eventos de válvula, presión de blowdown, retardo por longitud del colector). No reproduce la *sintonía por ondas*: reflexiones, barrido por resonancia o interacción en el colector. Hay además un fallo que no debes copiar. El retardo se dimensiona a 10.000 Hz fijos, pero recibe una muestra por paso físico, así que solo es correcto si `simulation_frequency = 10000`. Con 20 kHz, el retardo real en segundos se reduce a la mitad (inferencia a partir del código).

### Combustión, mecánica y la "chapuza" de la deriva

La combustión es un frente de llama cilíndrico. Arranca con la chispa si la riqueza equivalente está entre 0,5 y 1,9. Crece a una velocidad turbulenta: la velocidad laminar `S_L = S_L0·(T/298)^α·(P/1 atm)^β` multiplicada por un factor de turbulencia que depende de la velocidad media del pistón. A cada subpaso, la fracción de volumen recién barrida reacciona según `25 O₂ + 2 C₈H₁₆ → 16 CO₂ + 18 H₂O` ([fuel.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/fuel.cpp), [combustion_chamber.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/combustion_chamber.cpp)). Las constantes (φ_m = 1,21, B_m = 30,5 cm/s) coinciden con la correlación de **Metghalchi & Keck** para gasolina ([DOI 10.1016/0010-2180(82)90127-4](https://doi.org/10.1016/0010-2180(82)90127-4)). La atribución es una deducción a partir de los números, porque el código no la cita. Un término aleatorio (`rand()`) en la eficiencia de combustión produce la variación ciclo a ciclo que da el ralentí "irregular". Las pérdidas de calor del cilindro son una ley newtoniana muy burda hacia 90 °C.

La mecánica usa el solucionador de restricciones 2D del propio autor (**simple-2d-constraint-solver**). Monta pistones sobre líneas de cilindro, bielas con dos uniones y cigüeñales fijos con fricción. Las restricciones se resuelven con Gauss-Seidel (hasta 128 iteraciones, arranque en caliente), y la integración es Euler semi-implícito ([optimized_nsv_rigid_body_system.cpp](https://github.com/ange-yaghi/simple-2d-constraint-solver/blob/master/src/optimized_nsv_rigid_body_system.cpp), [gauss_seidel_sle_solver.cpp](https://github.com/ange-yaghi/simple-2d-constraint-solver/blob/master/src/gauss_seidel_sle_solver.cpp)). El código incluye un paso llamado "Correct drift (temporary hack)" que copia el ángulo del cigüeñal de salida en los demás ([simulator.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/simulator.cpp)). Es el síntoma de usar restricciones blandas para algo que en un motor en línea o en V se resuelve de forma exacta y analítica. El dinamómetro es una restricción de velocidad con límites de par, y el par medido es la reacción de esa restricción ([dynamometer.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/dynamometer.cpp)).

### Cómo se fabrica el sonido

Por cada cilindro, la señal es `att³·1600·[(P_escape − 1 atm) + 0,1·presión dinámica(+x) + 0,1·presión dinámica(−x)]`. Después se retarda por la longitud del escape, se escala por 1/L² y se suma en **un canal por sistema de escape** ([piston_engine_simulator.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/piston_engine_simulator.cpp)). El sintetizador aplica, en este orden ([synthesizer.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/synthesizer.cpp)):

1. Interpolación lineal a 44,1 kHz con un **Butterworth de 4.º orden a 1,9 kHz**.
2. Un filtro de "jitter" con retardo fraccional aleatorio.
3. Eliminación de la componente continua (DC) restando un paso bajo de 10 Hz.
4. Una mezcla de la señal modulada por ruido de aire filtrado a 2 kHz con un 1 % de su **derivada**.
5. **Convolución FIR directa** con una IR grabada en WAV, de hasta 10.000 coeficientes ([convolution_filter.cpp](https://github.com/ange-yaghi/engine-sim/blob/master/src/convolution_filter.cpp)).
6. Antialias a 0,45·fs.
7. Un **control automático de ganancia** (AGC).

Que el paso bajo esté en 1,9 kHz significa que la física solo aporta la parte grave del espectro. El brillo y la "aspereza" vienen de la IR, el ruido y el jitter. El AGC, por su parte, normaliza parte de la diferencia física de volumen entre carga y retención (inferencia). Un artículo de terceros afirma que el escape se modela como línea de transmisión y que hay una ruta de audio de admisión ([Starlog](https://starlog.is/articles/developer-tools/ange-yaghi-engine-sim)), pero **el código no lo confirma**, así que ese artículo no es fiable en esos puntos.

### El nuevo Engine Simulator ya es 1D, pero cerrado

El repositorio original redirige ahora a una "Community Edition" (v0.1.14a) que el propio README declara **sin mantenimiento**. Ange explica que dejó el código abierto porque "certain very high profile game studios" usaban su trabajo como propio ([engine-sim-community-edition](https://github.com/Engine-Simulator/engine-sim-community-edition)). El nuevo simulador está hecho sobre Vulkan y se describe como "a rigid-body mechanical simulation with a one-dimensional fluid simulation". Tiene página en Steam (app 2381500), **sin fecha de lanzamiento**, y un precio esperado de unos 10–20 $ ([Steam Community](https://steamcommunity.com/app/2381500)). Los vídeos de desarrollo son la mejor documentación técnica pública del salto a 1D, al menos por los títulos de sus capítulos:

| Fecha | Vídeo | Capítulos relevantes |
|---|---|---|
| 2 ago 2022 | [Simulating an Entire Car Engine (yes, it makes noise)](https://www.youtube.com/watch?v=RKT-sKtR970) | Lanzamiento del simulador 0D abierto |
| 29 ago 2022 | [Simulating VTEC, a radial-5 engine and more](https://www.youtube.com/watch?v=WPCMNnM6D0k) | VTEC, radiales |
| 26 sep 2022 | [Simulating an F1 V12, cross-plane and flat-plane V8s, unequal length headers](https://www.youtube.com/watch?v=NBBwva3NZ6o) | Colectores de longitud desigual |
| 7 sep 2023 | [Real-time Trumpet Simulation [C++/Vulkan]](https://www.youtube.com/watch?v=rGNUHigqUBM) | "The Euler Equations", "Tubular Fluid Engine", "Shock Tube Demo", "Reflective/Outflow Boundary" |
| 24 sep 2023 | [Initial Audio Testing – Devlog 3](https://www.youtube.com/watch?v=FJatcAkC8XI) | Primer uso del fluido 1D en motores |
| 31 oct 2023 | [Real-time Engine Tuning – Devlog 4](https://www.youtube.com/watch?v=-XHnDPCgIWk) | Sección variable, sintonía de escape |
| 3 may 2024 | [Multicylinder Engine Simulation – Devlog 6](https://www.youtube.com/watch?v=J11c8mMN1PA) | "Performance Issues", "Optimizations" con 4 cilindros |
| 25 oct 2025 | [Improving My Engine Sound Simulator With Real Data](https://www.youtube.com/watch?v=sUdnJTC2w9I) | Solucionadores de alta resolución, válvulas, calor y fricción, turbulencia, uniones de tubos |
| 5 may 2026 | [Creating Engine Simulator's Engine Editor](https://www.youtube.com/watch?v=82PUQ3Of9JM) | "Performance Considerations" |

En la descripción del vídeo de la trompeta, Ange recomienda *Internal Combustion Engine Fundamentals* (Heywood) y una serie sobre el método de las características. Las notas de investigación no incluyen transcripciones, así que el esquema numérico concreto, el número de celdas y las tasas de muestreo del nuevo solucionador **no están documentados**.

## Las tuberías 1D son el salto que da la sintonía del escape

### Una escala de fidelidad de cuatro peldaños

La literatura de motores ofrece cuatro niveles de modelo de gas, con costes muy distintos:

1. **Modelos de valor medio (MVEM)**, el más barato. Hendricks & Sorenson usan tres ecuaciones diferenciales sin resolución angular y logran **±2 % de precisión en estacionario**, comparable a una medida de banco ([SAE 900616](https://doi.org/10.4271/900616)). No dan pulsos, así que no sirven para generar sonido.
2. **Llenado-vaciado resuelto por ángulo de cigüeñal**: plenums con ecuaciones diferenciales y flujo de orificio en las válvulas. Es exactamente engine-sim.
3. **Modelo cuasi-propagatorio (QPM)** de Cipollone & Sciarretta, que "fills the gap existing between lumped-parameter and distributed-parameter models" y añade propagación dentro de un marco concentrado ([SAE 2001-01-0579](https://doi.org/10.4271/2001-01-0579)). Honda lo usó en 2023 para HILS en tiempo real ([JSAE 20239001](https://tech.jsae.or.jp/paperinfo/en/content/conf2023-02.01/)).
4. **Volúmenes finitos cuasi-1D completos**.

Los modelos industriales de tiempo real resueltos por ángulo comparten una misma idea: cilindros resueltos por ángulo y una red de tubos simplificada o más gruesa. Así lo hacen Ricardo WAVE-RT ([Pacitti et al., SAE 2008-01-1006](https://doi.org/10.4271/2008-01-1006)), AVL ([Wurzenberger et al., SAE 2009-01-0589](https://doi.org/10.4271/2009-01-0589)), GT-POWER con FEV xMOD ([Xia et al., SAE 2018-01-1245](https://doi.org/10.4271/2018-01-1245)) y los GT-SUITE Fast Running Models ([Andric et al., SAE 2018-01-0874](https://doi.org/10.4271/2018-01-0874)). Ninguno de los resúmenes accesibles da el Δt ni el Δx que usan.

### Qué esquema numérico elegir

La historia de los códigos de motor pasa por tres etapas. Primero, el método de las características (MOC) de Benson ([Benson, Garg & Woollatt 1964](https://doi.org/10.1016/0020-7403(64)90009-8)). Después, Lax-Wendroff de dos pasos con FCT o TVD en los años 90. Por último, solucionadores de Godunov tipo **HLLC**, como el código GASDYN del Politecnico di Milano ([Montenegro et al., SAE 2007-01-0495](https://doi.org/10.4271/2007-01-0495)). Las comparaciones coinciden en dos cosas:

- **Liu et al.** encontraron que todos los esquemas funcionan bien en flujo estacionario en tubos rectos. En flujo no estacionario, "especially in tapered pipes", todos tienen problemas de conservación de masa y energía, sobreoscilaciones y estabilidad ([SAE 960078](https://doi.org/10.4271/960078)).
- **Onorati, Winterbone & Pearson** mostraron con espectros FFT que Lax-Wendroff mete más contenido de alta frecuencia que el MOC ([SAE 930428](https://doi.org/10.4271/930428)). Para un simulador cuyo producto es el sonido, esto es decisivo: **la disipación y la dispersión del esquema se oyen**.

La referencia para implementar HLLC, MUSCL y TVD es el libro de Toro ([DOI 10.1007/b79761](https://doi.org/10.1007/b79761)), con los artículos originales de HLLC ([Toro et al. 1994](https://doi.org/10.1007/BF01414629); [Toro 2019](https://doi.org/10.1007/s00193-019-00912-4)).

Cualquier esquema explícito obedece la condición CFL, Δt ≤ C·Δx/max(|u|+a). OpenWAM la implementa como `FDeltaTime = FCourant * FXref / VTotalMax` ([OpenWAM](https://github.com/CMT-UPV/OpenWAM)). El grupo CMT de la Universitat Politècnica de València (UPV) atacó el cuello de botella resultante, porque los conductos cortos obligan a un paso global minúsculo. Su solución es una **discretización temporal independiente**, en la que cada elemento avanza con su propio límite de estabilidad ([Galindo et al. 2009, JEGTP](https://doi.org/10.1115/1.2983015); respuesta en alta frecuencia en [Math. Comput. Model. 2009](https://doi.org/10.1016/j.mcm.2009.05.011)). Los métodos de paso local de alto orden con corrección de flujos en la interfaz conservan la masa de forma exacta ([Hoang et al.](https://people.math.sc.edu/wangzhu/reference/Hoang2020MCOM190522-LTS_CLs.pdf); [Tan et al. 2004](https://www.math.hkbu.edu.hk/~ttang/Papers/localstep.pdf)). Hay una alternativa espectral: colocación ortogonal sobre características, que usa pocos nodos y corre en tiempo real ([Keblawi & McPhee, SAE 2019-01-0190](https://doi.org/10.4271/2019-01-0190)). Es menos adecuada donde hay choques o discontinuidades de contacto.

Las cifras que siguen son una estimación propia de las notas, no de un artículo. Con un escape caliente (a ≈ 600–700 m/s), celdas de 20–30 mm y Courant 0,8–0,9, el paso CFL ronda **20–30 µs**, del orden de la frecuencia de audio. Cuatro cilindros con unos 20–40 celdas por runner y primario suman 200–300 celdas, unos **10⁷ actualizaciones de celda por segundo** y del orden de **1–2 GFLOP/s en un núcleo**. Es viable en un ordenador de sobremesa, pero hay que medirlo antes de comprometerse con el diseño.

### Código abierto para aprender: ensim4 primero, OpenWAM después

**ensim4** de Gustav Louw es la referencia más útil para ti ([github.com/glouw/ensim4](https://github.com/glouw/ensim4)). Se describe como "a real-time inline combustion engine test-bed simulator that uses isentropic flow equations and one dimensional computational fluid dynamics to simulate exhaust gas pipe dynamics". Sus características principales ([wave_s.h](https://github.com/glouw/ensim4/blob/master/src/wave_s.h), [std.h](https://github.com/glouw/ensim4/blob/master/src/std.h)):

- Licencia MIT; último commit en noviembre de 2025.
- Unas 4.400 líneas de C, según las notas.
- Todo el motor avanza **a 48 kHz**, a la frecuencia de audio.
- El tubo de escape tiene 128 celdas con 8 subpasos (384 kHz), γ = 1,31 y un flujo **Rusanov / Lax-Friedrichs local**.
- Un micrófono virtual lee la presión en un punto del tubo, y una tecla activa o desactiva el CFD 1D para comparar.

Su predecesor, **ensim3**, es GPLv3 y organiza los nodos como volúmenes conectados en un grafo ([ensim3](https://github.com/glouw/ensim3)).

**OpenWAM** (CMT-UPV) es el único código 1D de motores maduro, validado y abierto ([GitHub](https://github.com/CMT-UPV/OpenWAM), [SourceForge](https://sourceforge.net/projects/openwam/)). Implementa Lax-Wendroff, MacCormack, CE-SE, TVD y FCT, junto con unos 20 tipos de condición de contorno: extremo abierto, cerrado y anecoico, válvulas, uniones, plenums, pérdidas de carga y conexión CFD. Las cabeceras declaran **GPLv3 o posterior**, el último commit es de febrero de 2017 y buena parte de los identificadores están en español. Si copias su código, tu proyecto queda obligado a ser compatible con la GPL. Leerlo para entender los algoritmos y reimplementarlos desde los artículos evita ese problema.

**aeolus1d** (Python, MIT, v0.4.2 de octubre de 2026) es una red de tubos con HLLC, MUSCL o MP5, SSP-RK y métodos implícitos. Sirve como plantilla de arquitectura, pero su validación es limitada ([PyPI](https://pypi.org/project/aeolus1d/)).

### Condiciones de contorno y validación

Las uniones de tubos tienen modelos no isentrópicos con pérdida de presión validados por el grupo de Blair en Queen's University Belfast ([SAE 940395](https://doi.org/10.4271/940395), [SAE 982055](https://doi.org/10.4271/982055)). Los cambios de sección se tratan en [SAE 932399](https://doi.org/10.4271/932399). Para validar tu código existen datos experimentales de tubos con discontinuidades y de códigos 1D ([SAE 941685](https://doi.org/10.4271/941685), [SAE 950276](https://doi.org/10.4271/950276)), además del caso de tubo cónico más volumen de Liu et al. (1996). El extremo abierto merece atención especial porque es la frontera que "suena". La solución exacta para un tubo sin brida es la de Levine & Schwinger ([Phys. Rev. 73, 1948](https://doi.org/10.1103/PhysRev.73.383)). Silva et al. dan aproximaciones cerradas, en acceso abierto, aptas para diseñar un filtro IIR ([arXiv:0811.3625](https://ar5iv.labs.arxiv.org/html/0811.3625)). Payri, Desantes & Torregrosa muestran cómo integrar un elemento acústico lineal, como la impedancia de un silenciador, como condición de contorno de un cálculo 1D no lineal ([JSV 188, 1995](https://doi.org/10.1006/jsvi.1995.0580)).

## La termodinámica de manual cuesta poco en tiempo real

### Combustión: Wiebe como base, un modelo cuasi-dimensional como mejora

La función de **Wiebe**, x_b = 1 − exp[−a((θ−θ₀)/Δθ)^(m+1)], es la elección estándar y barata. Ghojel hace una revisión histórica completa ([IJER 2010](https://doi.org/10.1243/14680874JER06510)). Hellstrom et al. explican cómo elegir *a*: el 99,9 % de Wiebe da a ≈ 6,908, y un 95 % da ≈ 2,996 ([PDF abierto](https://public.websites.umich.edu/~annastef/papers_hcci/HellstromASME2013ICEFall.pdf)). El código del libro de Ferguson & Kirkpatrick usa a = 5, n = 3, inicio en −20° y duración de 40° ([WoschniHeatTransfer.m](https://www.engr.colostate.edu/%7Eallan/matlab/WoschniHeatTransfer.m)). Con Wiebe, los fenómenos interesantes salen casi solos (inferencia de las notas):

- **Fallo de encendido:** basta con anular la energía liberada en ese ciclo.
- **Avance de encendido:** se obtiene haciendo θ₀ = chispa + retraso.
- **Carga:** sale de que la duración Δθ dependa de régimen, λ, gases residuales y presión de admisión.

Para que el motor responda "solo" al avance y a la turbulencia, el siguiente nivel es el modelo de arrastre turbulento de **Blizard–Keck / Tabaczynski** ([SAE 740191](https://doi.org/10.4271/740191), [SAE 770647](https://doi.org/10.4271/770647)). Sigue siendo un puñado de ecuaciones diferenciales sin malla espacial. El frente de llama de engine-sim es una versión simplificada de esta misma idea. La detonación se puede añadir barata con la integral de Livengood–Wu ([1955](https://doi.org/10.1016/S0082-0784(55)80047-1)) sobre el retraso de Douaud–Eyzat ([SAE 780080](https://doi.org/10.4271/780080)). La forma implementada en un ejemplo de ANSYS es τ = 0,01768·(ON/100)^3,402·p^−1,7·exp(3800/T) ([manual UDF de Fluent](https://www.afs.enea.it/project/neptunius/docs/fluent/html/udf/node40.htm)), pero **sus unidades no están verificadas**. Para diésel, las referencias son el doble Wiebe de Watson et al. ([SAE 800029](https://doi.org/10.4271/800029)) y Miyamoto et al. ([SAE 850107](https://doi.org/10.4271/850107)).

### Calor, propiedades del gas y válvulas

El coeficiente de **Woschni** ([SAE 670931](https://doi.org/10.4271/670931)) es una sola expresión algebraica por paso. Hay que vigilar dos cosas:

- **Unidades:** la constante 3,26 va con la presión en kPa, y 129,9 con la presión en bar. Mezclarlas introduce un error de ×10^0,8.
- **Exponente de la temperatura:** el código de Ferguson & Kirkpatrick usa T^−0,55, mientras que el original y otras implementaciones usan T^−0,53 ([libICEpost](https://libice-post.readthedocs.io/en/latest/autoapi/libICEpost/src/engineModel/HeatTransferModel/Woschni/index.html); [Trung 2021, acceso abierto](https://doi.org/10.24425/ather.2021.138114)).

Hohenberg ([SAE 790825](https://doi.org/10.4271/790825)) es la alternativa para diésel.

Para las propiedades del gas, la referencia abierta son los **coeficientes NASA Glenn**: más de 2.000 especies entre 200 y 20.000 K ([NASA/TP-2002-211556](https://ntrs.nasa.gov/citations/20020085330)). Un γ constante solo vale para un simulador "de sensación", porque sobrestima el rendimiento y la temperatura de pico. engine-sim usa γ = 1,4 fijo. Las notas proponen un truco barato: precalcular tablas de c_p, h y u para la mezcla fresca y los productos quemados, e interpolar en tiempo de ejecución.

Toda restricción al flujo (mariposa, válvulas, wastegate, EGR) se modela con la ecuación de orificio compresible con bloqueo y un área efectiva C_d·A(alzada). Los datos clásicos de C_d frente a alzada son de Kastner et al. ([Proc. IMechE 1963](https://doi.org/10.1243/PIME_PROC_1963_178_063_02)). Un trabajo abierto indica que C_d apenas depende del número de Reynolds para L/D ≈ 0,12–0,15 ([Soriano et al., ENCIT 2012](https://www.abcm.org.br/anais/encit/2012/links/pdf/ENCIT2012-158.pdf)).

### Fricción y mecánica: sin solucionador de restricciones

Chen–Flynn, FMEP = a·PCP + b·V_p + c·V_p² + d, es el modelo de fricción sensible a la carga más simple ([SAE 650733](https://doi.org/10.4271/650733); forma en [US 8006656](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8006656)). Funciona mal con encendidos muy retrasados ([Pipitone, SAE 2009-01-1984](https://doi.org/10.4271/2009-01-1984)). El modelo por componentes de Patton–Nitschke–Heywood, en su versión mejorada, tiene las ecuaciones completas en acceso abierto en la tesis de Sandoval en el MIT ([PDF DSpace](https://dspace.mit.edu/bitstream/1721.1/80657/2/52940875-MIT.pdf)), aunque la extracción de texto deja términos ilegibles que **hay que verificar**. Las pérdidas de bombeo salen solas del ciclo p–V si modelas los colectores, así que no hay que sumarlas dos veces.

Para la mecánica, todas las notas llegan a la misma conclusión. Un **modelo biela-manivela analítico de 1 grado de libertad** es exacto, sin ramas, determinista y de coste O(cilindros). Tiene tres piezas:

- La inercia equivalente varía con el ángulo: J(θ) = J₀ + m_rec·(ds/dθ)².
- El par del gas es p·A_p·ds/dθ.
- La inercia alternativa genera el rizado de 2.º orden.

Rizzoni da el modelo canónico de dinámica del cigüeñal ([IEEE TVT 1989](https://doi.org/10.1109/25.45470)). Un solucionador general solo compensa si quieres mecanismos arbitrarios: radiales con biela maestra, varios cigüeñales o embragues, que es lo que justifica el de engine-sim. Los textos de Eriksson & Nielsen ([Wiley 2014](https://doi.org/10.1002/9781118536186)) y Guzzella & Onder ([Springer 2010](https://doi.org/10.1007/978-3-642-10775-7)) cubren la transmisión, el vehículo, los receptores y la ecuación de la mariposa.

## El sonido sale de pulsos, tubos y una convolución

### Cuatro familias, y solo una encaja con tu simulador

La síntesis de sonido de motor se divide en cuatro familias:

1. **Muestras o granular síncrono con el tono**, como Crankcase REV y AudioMotors, que es lo que usan casi todos los juegos ([Designing Sound 2014](https://designingsound.org/2014/08/11/vehicle-engine-design-project-cars-forza-motorsport-5-and-rev/); [Jagla et al., JASA 2012](https://doi.org/10.1121/1.4754663)).
2. **Síntesis aditiva por órdenes**, usada en el diseño activo de sonido de coches eléctricos ([Cao et al., SAE 2020](https://doi.org/10.4271/10-04-02-0011)).
3. **Modelos procedurales con base física**: generadores de pulsos más guías de onda digitales (Farnell 2010; Baldan et al. 2015).
4. **Modelos neuronales y DDSP** de 2025–2026.

BeamNG, por ejemplo, documenta un sistema basado en capas de muestras mezcladas por RPM y carga, no en física de combustión ([BeamNG docs](https://documentation.beamng.com/modding/vehicle/sections/sounds/engine_audio/)). El veterano de Forza y Gran Turismo Mike Caviezel describe en 2026 la práctica industrial: bucles grabados cada ~500 RPM y cuatro perspectivas (escape, vano motor, admisión, interior). También subraya que el comportamiento con y sin carga importa tanto como las RPM ([BOOM Library](https://www.boomlibrary.com/blog/the-car-engine-sound-primer-mike-caviezel/)). Las notas añaden una interpretación: la industria usa muestras por control de autoría con coches licenciados, no por límites de CPU. El valor de un sintetizador físico está en lo que las grabaciones no cubren: geometría personalizada, fallos de encendido, petardeos y cambios de reglaje.

**Baldan et al. (2015)** es el artículo procedural más aplicable, y tiene postprint abierto ([PDF IUAV](https://air.iuav.it/retrieve/de164c2a-5461-60ee-e053-3a05fe0a7787/SIVE15_submission_4.pdf); [DOI 10.1109/SIVE.2015.7361287](https://doi.org/10.1109/SIVE.2015.7361287)). Su estructura:

- Cada cilindro es una guía de onda cuyo retardo modula el pistón.
- Las válvulas modulan la realimentación.
- La admisión, los colectores y el escape son guías de onda.
- El silenciador son cuatro guías de onda paralelas parcialmente reflectantes.
- Los petardeos son impulsos estocásticos.
- Hay tres salidas: admisión, bloque y escape.

Su implementación abierta es el módulo **SDTMotor** del Sound Design Toolkit ([API SDT](https://sdt.lim.di.unimi.it/docs/API/v2.2-078/group__motor.html)). El proyecto Rust **DasEtwas/enginesound** (MIT) se basa "loosely" en este artículo, con cpal y configuración en RON ([GitHub](https://github.com/DasEtwas/enginesound)). La práctica 22 "Cars" de Farnell en *Designing Sound*, en Pure Data, es el antecedente ([MIT Press](https://mitp-content-server.mit.edu/books/content/sectbyfn/books_pres_0/8375/designing_sound.zip/practical22.html)). En tu caso, la excitación con semisenos dibujados a mano de Baldan se sustituye por los pulsos de presión reales de tu simulación. Lo reutilizable es la red de guías de onda y la estructura de varias salidas.

En la familia neuronal, el modelo PTR de **Doerfler & Wyse** combina tres ingredientes, todos en acceso abierto ([arXiv:2603.09391](https://arxiv.org/abs/2603.09391)):

- trenes de pulsos alineados con el orden de encendido;
- una velocidad del sonido ligada a la temperatura del gas;
- resonadores Karplus-Strong parametrizados por coeficientes de reflexión.

Mejora un 21 % la reconstrucción armónica frente a un modelo armónico más ruido, pero **no publica cifras de tiempo real**. Sirve como fuente de parametrizaciones con motivación física, no como sustituto de un núcleo físico.

### De la presión simulada a 48 kHz

Las guías de onda vienen del libro gratuito *Physical Audio Signal Processing* de Julius O. Smith ([PASP](https://ccrma.stanford.edu/~jos/pasp/)):

- Un tubo uniforme son dos líneas de retardo opuestas.
- Un cambio de sección es una unión de Kelly-Lochbaum ([sección](https://ccrma.stanford.edu/~jos/pasp/Kelly_Lochbaum_Scattering_Junctions.html)).
- Las longitudes no enteras usan interpolación de Lagrange o pasa-todo de Thiran ([sección](https://ccrma.stanford.edu/~jos/pasp/Delay_Line_Interpolation.html); [Laakso et al. 1996](https://doi.org/10.1109/79.482137)). Las necesitas porque la velocidad del sonido cambia con la temperatura del escape.
- Un resonador lateral, como un Helmholtz o un silenciador de derivación, tiene la misma topología que el agujero de un clarinete ([sección](https://ccrma.stanford.edu/~jos/pasp/Tonehole_Modeling.html)).

El coste es de unas pocas multiplicaciones por unión y muestra. Hay un límite: las guías de onda son acústica lineal y no capturan el empinamiento de pulsos grandes ni el flujo medio. Por eso encajan **aguas abajo** del solucionador no lineal (silenciador y cola) o como sustituto barato en el nivel de calidad bajo.

En la salida, un monopolo radia una presión proporcional a la **derivada temporal del caudal volumétrico** ([PASP, fuente puntual](https://ccrma.stanford.edu/~jos/pasp/Spherical_Waves_Point_Source.html)). Pasar el caudal de salida por un derivador es más físico que la mezcla perceptual de engine-sim (presión menos atmosférica más un 1 % de derivada). Haz lo que hagas, el filtro de DC ([JOS, DC Blocker](https://ccrma.stanford.edu/~jos/filters/DC_Blocker.html)) debe cortar en pocos hercios: los motores con pocos cilindros tienen fundamentales de **~5 Hz** al ralentí ([tesis de Doerfler 2025](https://recercat.cat/handle/10230/72480)). Para el cambio de frecuencia de muestreo, la interpolación lineal más un IIR no está limitada en banda. La opción limpia es el remuestreo sinc enventanado de Smith ([JOS resample](https://ccrma.stanford.edu/~jos/resample/)). La convolución directa de engine-sim cuesta hasta 10.000 multiplicaciones-suma por muestra y canal (~441 M/s). La **convolución FFT con particionado uniforme** de Wefers & Vorländer ([DAFx-11, abierto](https://www.dafx.de/paper-archive/2011/Papers/74_e.pdf)) la reduce en órdenes de magnitud. La versión no uniforme de Gardner ([JAES 1995](https://aes2.org/publications/elibrary-page/?id=7957)) además elimina la latencia.

### Por qué los fluidos de videojuego no te sirven

Los solucionadores de Stam ([Stable Fluids 1999](https://doi.org/10.1145/311535.311548); [GDC 2003, abierto](https://www.dgp.toronto.edu/public_user/stam/reality/Research/pdf/GDC03.pdf)), la mayoría de los eulerianos de gráficos y el SPH incompresible imponen ∇·u = 0. Eso equivale a una velocidad del sonido infinita, así que **no propagan ondas acústicas**. El SPH débilmente compresible de Müller usa una velocidad del sonido artificial ([SCA 2003](https://matthias-research.github.io/pages/publications/sca03.pdf)). La advección semi-lagrangiana es estable pero difusiva, lo cual es perfecto para humo y fatal para ondas de varios kHz. Todo esto es inferencia razonada de las notas.

El sonido a partir de CFD en gráficos se calcula offline. Dobashi et al. precalculan "sound textures" ([TOG 2003](https://doi.org/10.1145/882262.882339)), con 100–155 min de CFD en 32 núcleos por muestra según un informe de Stanford ([Wang](https://web.stanford.edu/~jw969/assets/projects/aerosound/aerosound.pdf)). Wang et al. 2018 apuntan explícitamente a síntesis offline en la nube ([TOG 2018](https://doi.org/10.1145/3197517.3201318)). El método de Lattice Boltzmann puede transportar acústica, pero a coste de investigación en aeroacústica ([Shao & Li 2019, abierto](https://doi.org/10.24425/aoa.2019.128486)).

Que 1D baste tiene una explicación física, aunque la fórmula no está verificada contra una fuente en las notas. Por debajo del primer modo transversal de un conducto circular (≈ 1,84·c/(π·D)) solo se propagan ondas planas. Para un tubo de 50 mm eso son ≈ 4 kHz en frío y ≈ 5,9 kHz en caliente. Los armónicos de encendido dominantes están muy por debajo de esos valores. El CFD 3D solo es útil **offline**, para hornear IRs, curvas de pérdida de transmisión o coeficientes de pérdida en uniones.

## En máquinas lentas pesa más la arquitectura que los FLOPs

### Pasos fijos y pequeños, y calidad adaptativa

Un simulador de motor tiene pocos elementos pero muchísimos pasos: 4–12 cilindros, decenas o cientos de celdas y 10.000–100.000 pasos/s. El coste por paso (llamadas, despacho dinámico, asignaciones de memoria, fallos de caché) domina sobre los FLOPs brutos (inferencia de las notas). Hay tres razones para usar un paso fijo:

- Evita la "espiral de la muerte" y da determinismo bit a bit ([Fiedler, Fix Your Timestep!](https://gafferongames.com/post/fix_your_timestep/)).
- Muchos subpasos baratos ganan a pocos pasos grandes implícitos. En "Small Steps", 100 subpasos con 1 iteración resuelven una cadena con relación de masas 1:1000 en 6 ms, rígida y estable. Con 1 paso y 500 iteraciones se tarda 17 ms y la cadena sigue estirándose ([Macklin et al. 2019](https://matthias-research.github.io/pages/publications/smallsteps.pdf)).
- Los solucionadores adaptativos de EDO rígidas (como `diffsol`) tienen un coste variable por llamada, arriesgado en un bucle de tiempo real (inferencia). Para volúmenes diminutos, como el hueco de válvula, conviene tratarlos como orificios cuasiestacionarios sin estado o con un Euler implícito linealizado de coste fijo.

La palanca principal en máquinas lentas es la misma que recomienda la FAQ oficial de engine-sim: bajar la frecuencia de simulación, "but lowering it too much might cause the simulation to become unstable" ([wiki FAQ](https://github.com/ange-yaghi/engine-sim/wiki/Frequently-Asked-Questions)). El síntoma de "el sonido se repite" tiene una causa concreta. Cuando el búfer se vacía, engine-sim rellena con ceros y el búfer cíclico del dispositivo reproduce datos viejos. Tu diseño debería **automatizar esa palanca**: si el búfer circular tiende a vaciarse, reducir subpasos, celdas o nivel de modelo. Al reducir celdas en 1D, el coste baja de forma cuadrática, porque disminuyen a la vez las celdas y los pasos (inferencia).

### Hilos, búferes y reglas de audio en tiempo real

Las reglas de Bencina para el callback de audio son estas: nada de asignar memoria, bloquear mutex, hacer E/S ni ejecutar trabajo de duración desconocida. Lo resume así: "If you don't know how long it will take, don't do it" ([Bencina](http://www.rossbencina.com/code/real-time-audio-programming-101-time-waits-for-nothing)). engine-sim las incumple al tomar un `lock_guard` en la lectura del audio ([synthesizer.cpp](https://raw.githubusercontent.com/ange-yaghi/engine-sim/master/src/synthesizer.cpp)). En Rust:

- **`rtrb`** es un búfer circular SPSC sin espera que reserva toda su memoria al construirse ([docs](https://docs.rs/rtrb/latest/rtrb/)).
- **`triple_buffer`** sirve para compartir el "último valor" entre la interfaz y la simulación ([docs](https://docs.rs/triple_buffer/latest/triple_buffer/)).
- **`assert_no_alloc`** detecta asignaciones en zonas marcadas ([docs](https://docs.rs/assert_no_alloc/latest/assert_no_alloc/)).

En ALSA, `cpal` puede elegir búferes por defecto enormes, así que conviene fijar `BufferSize::Fixed(n)` ([cpal](https://github.com/RustAudio/cpal)). El paralelismo debe ser **grueso**: un hilo de simulación, uno de audio y uno de interfaz. Un fork/join con `rayon` en cada paso a 10–80 kHz cuesta más en sincronización que el trabajo que reparte. En un doble núcleo antiguo, lo mejor probablemente sea un único hilo de simulación (inferencia).

### CPU: SoA, SIMD con despacho, f64 donde importa y FTZ

Los tres primeros puntos se resumen en la [Rust Performance Book](https://nnethercote.github.io/perf-book/build-configuration.html):

- **Perfil de compilación:** LTO "can improve runtime speed by 10-20% or more", PGO un 10 % o más, y `codegen-units = 1`.
- **CPU de destino:** `target-cpu=native` produce binarios que pueden no funcionar en procesadores más antiguos.
- **Herramientas de perfilado:** perf, samply, flamegraph, Cachegrind, DHAT y Tracy.

`std::simd` sigue siendo solo de nightly ([docs](https://doc.rust-lang.org/std/simd/index.html)). En estable, **`pulp`** despacha en tiempo de ejecución según las capacidades de la CPU, que es lo que necesitas si distribuyes un binario para máquinas viejas. **`wide`** ofrece tipos de ancho fijo ([PythonSpeed](https://pythonspeed.com/articles/simd-stable-rust)). Las notas recomiendan tres cosas:

- **Estructura de arrays (SoA)** en las celdas del tubo (`rho[]`, `rho_u[]`, `E[]`) y vectorizar *entre cilindros*.
- **f64** para el estado termodinámico, porque se restan presiones grandes casi iguales, y f32 para el posprocesado de audio. Avian usa una estrategia mixta parecida con un coste de "just a few percent" ([avian3d](https://docs.rs/avian3d/latest/avian3d/)).
- **Vaciar los subnormales a cero (FTZ/DAZ)** en los hilos de audio y de simulación. Los subnormales disparan la CPU cuando las señales decaen ([Tytel](https://tytel.org/blog/2015/02/10/unexpected-optimization-01/)). `no_denormals` lo hace, aunque advierte que es "technically undefined behaviour" ([docs](https://docs.rs/no_denormals/latest/no_denormals/)).

## ¿1D o 2D? Viabilidad y coste (ESTIMACIÓN PROPIA)

Esta sección responde a si el simulador puede ser 1D, o incluso 2D, y seguir funcionando en tiempo real. Los números de coste son estimaciones de orden de magnitud hechas a partir de la condición CFL; no salen de ningún paper. Los datos de ensim4, Galindo 2009 y Montenegro 2013 sí están en la bibliografía.

### 1D: viable, y es el camino correcto

No es una apuesta. **ensim4** (C, MIT) ya corre un solver 1D de volúmenes finitos a 48 kHz, con 128 celdas, 8 subpasos y flujo de Rusanov. El nuevo Engine Simulator de Ange también es 1D. En la industria hay versiones 1D en tiempo real para bancos HIL (GT-POWER, WAVE-RT).

El coste lo marca la condición CFL: el paso de tiempo tiene que ser menor que el tamaño de celda dividido entre la velocidad de las ondas, Δt ≤ CFL·Δx / (|u| + c).

- **Velocidad de onda:** en el escape caliente, |u| + c ronda 700–800 m/s.
- **Paso de tiempo:** con Δx = 3 cm sale Δt ≈ 30 µs. Basta un paso por muestra de audio a 48 kHz, o dos para ir holgado.
- **Tamaño de celda necesario:** una onda de 2 kHz mide unos 30 cm en gas caliente, así que 3 cm dan unas 10 celdas por longitud de onda, suficiente. engine-sim corta en 1,9 kHz de todos modos; lo que va por encima se añade con la convolución o con ruido filtrado.
- **Número de celdas:**
  - Un 4 cilindros con runners, primarios, colector y cola tiene unos 8–10 m de tubo, unas 250–350 celdas.
  - Un V8 anda por las 400–500 celdas.
- **Coste por segundo:**
  - Son del orden de 10–50 millones de actualizaciones de celda por segundo.
  - Con HLLC + MUSCL, unas 100–200 operaciones por celda, salen unos pocos GFLOP/s.
  - Un núcleo de un portátil normal lo saca. En un doble núcleo viejo, un V8 a calidad máxima va justo; por eso hacen falta los niveles de calidad automáticos (fase 5 de la hoja de ruta).

Hay tres técnicas que importan más que cualquier micro-optimización:

1. **Segundo orden (MUSCL)** permite celdas más gruesas que Rusanov de primer orden, que además amortigua los agudos.
2. **Cada tubo con su propio paso de tiempo**, para que un runner corto no obligue a todo el motor a ir lento (Galindo et al. 2009).
3. **SIMD agrupando celdas de tubos distintos**, con despacho en tiempo de ejecución (`pulp`).

### 2D: existe, pero no compensa en tiempo real

La CFD compresible en 2D existe (Euler 2D, lattice Boltzmann). El problema es doble.

**Física.** Dentro de un conducto, por debajo de una frecuencia de corte solo se propagan ondas planas, y eso es exactamente lo que modela el 1D. El primer modo no plano de un conducto circular aparece en f_c = 1,84·c / (π·d):

| Conducto | Diámetro | Temperatura del gas | Corte aproximado |
|---|---|---|---|
| Primario de escape | 5 cm | caliente (~700 K) | ~6 kHz |
| Runner de admisión | 4 cm | ambiente | ~5 kHz |
| Silenciador | 20 cm | caliente | ~1,5 kHz |

En los tubos, el 2D no aporta nada audible. Donde sí aporta es en el silenciador y en las uniones bruscas. Además, un 2D plano ni siquiera es correcto para un tubo redondo, porque modela un canal; lo que tendría sentido es el 2D axisimétrico (r–z).

**Coste.** Un corte 2D del sistema de escape con celdas de 5 mm son unas 10.000 celdas, y el paso baja a unos 7 µs. Eso da del orden de 1.000 millones de actualizaciones por segundo, unas 50–100 veces el 1D. Con GPU (compute en `wgpu`) y leyendo el audio por bloques podría ir en una gráfica decente, pero en una máquina lenta no.

### Decisión: híbrido 1D + 2D offline

- **1D no lineal** (HLLC + MUSCL, paso por tubo) en runners, primarios, colector y cola.
- **Silenciador** como red de celdas 0D (Montenegro et al. 2013) o como matriz de transferencia (Munjal).
- **2D/3D solo offline:** simular el silenciador una vez con un solver 2D axisimétrico, guardar su respuesta al impulso o su matriz de transferencia, y usarla en tiempo real casi gratis. Puede ofrecerse como herramienta de diseño de escapes dentro del propio simulador.

**Primer paso recomendado:** antes de montar el resto, implementar un tubo 1D aislado en Rust (HLLC + MUSCL + SSP-RK2), validarlo con el tubo de choque de Sod y medir cuántas celdas por segundo saca la máquina más lenta objetivo. Ese número decide cuántos tubos y qué tamaño de celda permite cada nivel de calidad.

## Arquitectura y hoja de ruta recomendadas (RECOMENDACIÓN / INFERENCIA)

Esta sección no recoge hallazgos de fuentes. Es una síntesis propuesta a partir de las notas: lo que hacen engine-sim, ensim4 y los modelos industriales de tiempo real, más las estimaciones de coste marcadas como inferencia.

### Arquitectura de hilos y flujo de datos

La recomendación es un **hilo de simulación** que avance a paso fijo y produzca muestras ya procesadas. Su DSP incluye filtro de DC, derivador o radiación, convolución por FFT y remuestreo, y lo escribe todo en un `rtrb::Producer<f32>`. El **callback de `cpal`** solo saca muestras del búfer y aplica una ganancia final. Así no bloquea nunca y en depuración puede envolverse en `assert_no_alloc`. La **interfaz** (egui/eframe) envía mariposa, encendido y carga del dinamómetro con `triple_buffer` o una cola de comandos, y recibe instantáneas del estado por otro `triple_buffer`. El audio es el reloj maestro. Un controlador proporcional, como el de engine-sim pero sin mutex, ajusta los pasos por bloque según el llenado del búfer y apunta a 20–50 ms de latencia total, frente a los 100 ms de engine-sim. Ante un vaciado del búfer, se mantiene o se funde la última muestra en lugar de rellenar con ceros, y si el problema persiste se baja un nivel de calidad. Para un determinismo trivial, conviene que la frecuencia de simulación sea un múltiplo entero de la de audio: 48 kHz como ensim4, o 24 o 96 kHz.

### Fases de desarrollo

| Fase | Objetivo | Modelo y técnicas | Referencias principales | Criterio de "hecho" |
|---|---|---|---|---|
| 0 | Tubería de audio | cpal + rtrb + triple_buffer; síntesis procedural tipo Baldan/SDT como señal de prueba; FTZ; perfil de compilación con LTO | Bencina; Baldan 2015; PASP | Sin cortes con un búfer de 256–1024 muestras en tu máquina más lenta |
| 1 | Monocilindro 0D | Biela-manivela analítica de 1 GDL; volúmenes 0D (runner de admisión, cilindro, runner de escape, colector) con orificio compresible y C_d(alzada) por tabla; Wiebe con fallo y avance; Woschni; γ constante → tablas NASA; RNG con semilla para la variación ciclo a ciclo | Código de engine-sim (`gas_system`, `combustion_chamber`); Heywood; Ferguson & Kirkpatrick; NASA TP-2002-211556 | Par y p–V coherentes; ralentí estable; sonido de "un cilindro" |
| 2 | Multicilindro y carga | Orden de encendido, bancadas en V con desfase por bancada; Chen–Flynn o fricción por componentes (Sandoval); dinamómetro con PI de velocidad; embrague y marchas; motor de arranque | Rizzoni 1989; Eriksson & Nielsen; tesis de Sandoval | Curva de par en dinamómetro; sonido de 4/6/8 cilindros comparable a engine-sim |
| 3a | Escape barato con ondas | Guías de onda: primarios → unión de N puertos ponderada por área → tubo → silenciador (guías paralelas o escalones de Kelly-Lochbaum) → filtro de radiación (ajuste de Levine–Schwinger, Silva et al.) | PASP; Baldan 2015; enginesound; arXiv:0811.3625 | Las longitudes de los primarios cambian el timbre de forma audible |
| 3b | Escape 1D no lineal | Volúmenes finitos cuasi-1D: Rusanov (como ensim4) → HLLC + MUSCL (minmod/van Leer) + SSP-RK2; término de área bien equilibrado; válvula como frontera de orificio; extremo abierto con radiación; uniones de Blair/Fleck; subciclado por tubo | Toro; ensim4; Galindo 2009; Montenegro 2007; Blair SAE 940395/982055 | Pasa el tubo de choque de Sod y el tubo cónico de Liu 1996; la sintonía de colectores se oye |
| 4 | Acústica de salida | Monopolo d(ρuA)/dt en la cola; silenciador como red cuasi-3D de celdas 0D o como filtro TMM; IR opcional por FFT particionada para punta, carrocería, micro y sala; varias perspectivas (escape, admisión, bloque, interior) | Munjal; Montenegro 2013 (abierto); Payri 1995; Wefers & Vorländer 2011 | IR de 10.000 coeficientes con un coste despreciable |
| 5 | Optimización y calidad adaptativa | SoA; SIMD entre cilindros con despacho `pulp`; f64 en el estado y f32 en el audio; niveles de calidad automáticos según el llenado del búfer: (1) FV 1D completo → (2) malla gruesa → (3) 0D + guías de onda → (4) mapas + síntesis procedural; PGO | perf-book; Small Steps; Fiedler | Funciona en un portátil viejo de doble núcleo con el nivel 3 sin cortes |

Tres reglas recorren todas las fases. Primero, **corrige los fallos conocidos de engine-sim al portar**: retardo calculado con el paso real, filtros por canal (engine-sim usa siempre el filtro del canal 0 para el ruido de aire) y arrays dimensionados para más de 8 cilindros. Segundo, **define los motores con datos**, en `serde` + `ron` o `toml`, en lugar de un lenguaje propio como el Piranha de engine-sim, y añade `rhai` solo si necesitas guiones. Tercero, **valida contra referencias publicadas** antes de ajustar de oído: el caso de Liu 1996, los datos experimentales de QUB y la propia salida de engine-sim y ensim4 para el mismo motor.

Sobre licencias: engine-sim, ensim4 y enginesound son MIT y su código se puede portar con atribución. OpenWAM, ensim3, ghurni y el banco de pruebas TCSI de la Universidad de Linköping (LiU) son GPL-3, así que es mejor leerlos y reimplementar. El modelo PTR de Doerfler & Wyse es CC BY-NC-SA, que no permite uso comercial.

## Crates de Rust (versiones de crates.io a 21-10-2026)

| Área | Crate | Versión | Última publicación | Uso propuesto / notas |
|---|---|---|---|---|
| E/S de audio | [`cpal`](https://crates.io/crates/cpal) | 0.18.2 | 2026-08-16 | E/S multiplataforma de bajo nivel; fijar `BufferSize::Fixed`. El aviso de xrun (`CallbackInfo::xrun`) solo aparece en "Unreleased" |
| E/S de audio | [`rodio`](https://crates.io/crates/rodio) | 0.23.0 | 2026-10-11 | Reproducción de alto nivel; innecesario si usas cpal |
| E/S de audio | [`tinyaudio`](https://crates.io/crates/tinyaudio) | 2.0.0 | 2025-11-15 | Alternativa mínima, solo salida |
| Motor de audio | [`kira`](https://docs.rs/kira/latest/kira/) | 0.12.5 | 2026-09-26 | Mezclador para juegos; útil si mezclas muchos sonidos |
| Motor de audio | [`firewheel`](https://docs.rs/crate/firewheel/latest/source/README.md) | 0.14.0 | 2026-09-05 | Grafo de audio "no mutexes!"; futuro audio de Bevy |
| Motor de audio | [`oddio`](https://docs.rs/oddio/latest/oddio/) | 0.7.4 | 2023-10-15 | Estancado |
| DSP | [`fundsp`](https://github.com/SamiPerttu/fundsp) | 0.23.0 | 2026-01-07 | Grafos DSP, bloques de 64 con `wide::f32x8`; `convolve` vía fft-convolver |
| DSP | [`dasp`](https://crates.io/crates/dasp) | 0.11.0 | 2020-05-29 | Primitivas de muestras; sin publicaciones desde 2020 |
| Remuestreo | [`rubato`](https://crates.io/crates/rubato) | 5.0.1 | 2026-10-01 | Sinc o FFT síncrono y asíncrono; para pasar de la tasa de simulación a la de audio |
| FFT | [`rustfft`](https://crates.io/crates/rustfft) / [`realfft`](https://crates.io/crates/realfft) | 6.4.1 / 3.5.0 | 2025-09-18 / 2025-06-12 | FFT acelerada con SIMD |
| Convolución | [`fft-convolver`](https://crates.io/crates/fft-convolver) | 0.4.0 | 2026-06-15 | Convolución FFT particionada; sustituye al FIR directo |
| Subnormales | [`no_denormals`](https://docs.rs/no_denormals/latest/no_denormals/) | 0.3.0 | 2025-09-11 | FTZ/DAZ con guarda RAII; los docs avisan de UB técnico |
| Sin bloqueos | [`rtrb`](https://docs.rs/rtrb/latest/rtrb/) | 0.4.0 | 2026-08-18 | SPSC sin espera, simulación → audio |
| Sin bloqueos | [`ringbuf`](https://crates.io/crates/ringbuf) | 0.5.3 | 2026-10-08 | SPSC alternativo |
| Sin bloqueos | [`triple_buffer`](https://docs.rs/triple_buffer/latest/triple_buffer/) | 9.0.0 | 2026-02-22 | Parámetros de interfaz ↔ simulación |
| Seguridad RT | [`basedrop`](https://docs.rs/basedrop/latest/basedrop/) | 0.1.3 | 2025-10-29 | Liberación diferida fuera del hilo de audio |
| Seguridad RT | [`assert_no_alloc`](https://docs.rs/assert_no_alloc/latest/assert_no_alloc/) | 1.1.2 | 2021-08-03 | Antiguo pero muy usado |
| Matemáticas | [`glam`](https://docs.rs/glam/latest/glam/) | 0.34.1 | 2026-10-06 | Vectores SIMD; `core-simd` requiere nightly |
| Matemáticas | [`nalgebra`](https://crates.io/crates/nalgebra) / [`faer`](https://crates.io/crates/faer) | 0.35.0 / 0.24.4 | 2026-05-24 / 2026-06-24 | Álgebra lineal densa o dispersa (implícitos, validación) |
| SIMD | [`wide`](https://crates.io/crates/wide) | 1.7.1 | 2026-09-14 | Tipos de ancho fijo en estable |
| SIMD | [`pulp`](https://docs.rs/pulp/latest/pulp/) | 0.22.3 | 2026-06-20 | Despacho en tiempo de ejecución por CPU, clave para binarios en máquinas viejas |
| EDO | [`diffsol`](https://docs.rs/diffsol/latest/diffsol/) | 0.18.0 | 2026-10-20 | BDF/SDIRK para EDO rígidas; mejor offline o para validar |
| EDO | [`ode_solvers`](https://crates.io/crates/ode_solvers) | 0.6.2 | 2026-06-07 | Solo explícitos |
| Física | [`rapier3d`](https://docs.rs/rapier3d/latest/rapier3d/) | 0.36.1 | 2026-10-10 | Probablemente excesivo para el tren alternativo |
| Física | [`avian3d`](https://docs.rs/avian3d/latest/avian3d/) | 0.8.0 | 2026-10-20 | Subpasos y restricciones blandas; ídem |
| Interfaz | [`egui`/`eframe`](https://crates.io/crates/eframe) | 0.37.0 | 2026-10-15 | Relojes, gráficas y controles; ligero |
| Interfaz | [`macroquad`](https://crates.io/crates/macroquad) | 0.4.16 | 2026-07-30 | Alternativa ligera |
| Interfaz | [`bevy`](https://crates.io/crates/bevy) / [`wgpu`](https://crates.io/crates/wgpu) | 0.20.0 / 30.0.1 | 2026-10-08 / 2026-08-22 | Pesados para GPU antiguas |
| Configuración | [`serde`](https://crates.io/crates/serde) / [`ron`](https://crates.io/crates/ron) / [`toml`](https://crates.io/crates/toml) | 1.0.229 / 0.12.2 / 1.1.8 | 2026-07-18 / 2026-06-22 / 2026-10-09 | Definición declarativa de motores |
| Scripting | [`rhai`](https://crates.io/crates/rhai) / [`mlua`](https://crates.io/crates/mlua) | 1.27.0 / 0.12.2 | 2026-10-21 / 2026-10-03 | Motores con guiones, solo si hace falta |
| Perfilado | [`tracy-client`](https://crates.io/crates/tracy-client) / [`flamegraph`](https://crates.io/crates/flamegraph) | 0.19.0 / 0.6.14 | 2026-08-25 / 2026-08-12 | Perfilado por fotograma o paso |
| Bancos de pruebas | [`criterion`](https://crates.io/crates/criterion) / [`divan`](https://docs.rs/divan/latest/divan/) | 0.8.2 / 0.1.21 | 2026-02-04 / 2025-04-10 | Microbenchmarks del paso de celda |
| Hilos | [`rayon`](https://crates.io/crates/rayon) / [`crossbeam-channel`](https://crates.io/crates/crossbeam-channel) | 1.12.0 / 0.5.17 | 2026-04-14 / 2026-09-05 | No usar rayon por paso |
| Prior art | [`ghurni`](https://docs.rs/crate/ghurni/latest) | 1.0.0 | 2026-03-28 | Síntesis procedural de sonido mecánico; GPL-3.0; afirma ~1000× tiempo real |
| Prior art | [`lmb_engine_simulator`](https://www.lib.rs/crates/lmb_engine_simulator) | 0.1.0 | 2021 | Simulador termodinámico 0D, sin audio |

## Bibliografía anotada por temas

Clave de acceso: **Abierto** = copia legal gratuita. **De pago** = copia del editor de pago (SAE Mobilus, IEEE, Elsevier, Springer, Wiley, ACM, ASME, IMechE/SAGE, AES). **Código** = repositorio con su licencia. Los DOI se comprobaron en Crossref durante la investigación, salvo que se indique otra cosa.

### Simuladores y código de referencia

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Yaghi, A., engine-sim — [github.com/ange-yaghi/engine-sim](https://github.com/ange-yaghi/engine-sim); solucionador [simple-2d-constraint-solver](https://github.com/ange-yaghi/simple-2d-constraint-solver) | Código, MIT | La referencia de facto: modelo 0D, orificio compresible, frente de llama, cadena DSP. "NOT a scientific tool". Solo compila en Windows |
| Louw, G., ensim4 — [github.com/glouw/ensim4](https://github.com/glouw/ensim4); ensim3 — [github.com/glouw/ensim3](https://github.com/glouw/ensim3) | Código, MIT / GPLv3 | El mejor modelo a portar: todo a 48 kHz, tubo FV 1D con Rusanov, micrófono virtual, conmutador A/B del CFD |
| CMT-UPV, OpenWAM — [GitHub](https://github.com/CMT-UPV/OpenWAM), [SourceForge](https://sourceforge.net/projects/openwam/) | Código, GPLv3+ | Esquemas LW/MacCormack/CE-SE/TVD/FCT y unas 20 condiciones de contorno; inactivo desde 2017; identificadores en español |
| aeolus1d — [PyPI](https://pypi.org/project/aeolus1d/) | Código, MIT | Red de tubos HLLC/MUSCL con acoplamiento implícito; validación limitada |
| DasEtwas/enginesound — [GitHub](https://github.com/DasEtwas/enginesound) | Código, MIT (Rust) | Precedente en Rust: guías de onda por cilindro, silenciador, cpal y RON. Nivel de calidad bajo |
| Sound Design Toolkit, SDTMotor — [API](https://sdt.lim.di.unimi.it/docs/API/v2.2-078/group__motor.html) | Código (licencia no verificada) | Implementación abierta del modelo de Baldan 2015 |
| EngineLab — [GitHub](https://github.com/zolaski333/EngineLab) | Código (C++20/JUCE) | Conductos cuasi-1D, cámaras de Munjal, material poroso; un solo autor y 0 estrellas, sin validar |
| Ferguson & Kirkpatrick, códigos MATLAB — [Colorado State](https://www.engr.colostate.edu/~allan/links.html) | Abierto | Volume, BurnFraction, Woschni, propiedades del gas, Friction: pruebas unitarias para tu termodinámica |
| LiU TCSI Engine Simulation Testbed — [LiU](https://www.vehicular.isy.liu.se/Software/TCSI_Testbed); Ng et al., [IEEE CSM 2020](https://doi.org/10.1109/MCS.2019.2961793) | Código GPL-3 / artículo de pago | MVEM con turbo en Simulink; referencia para el turbo |
| Jonsson & Eriksson 2025, MVEM por componentes — [DOI 10.3384/ecp212.045](https://doi.org/10.3384/ecp212.045) | Abierto (CC BY 4.0) | Restricciones compresibles e incompresibles, VVA |
| Cantera, `ic_engine.py` — [ejemplo](https://www.cantera.org/examples/python/reactors/ic_engine.py.html) | Código abierto | Química detallada, no apta para tiempo real; útil para generar tablas de propiedades offline |
| "Revving: Engine Simulator" — [Steam](https://store.steampowered.com/app/5358500/) | Comercial | Juego que porta engine-sim bajo MIT: confirma que la licencia lo permite |
| Comunidad engine-sim — [wiki](https://github.com/ange-yaghi/engine-sim/wiki/Community-projects) | Código | Enginette, Mod Loader con Lua, Engine Sim Recorder |

### Libros de base

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Heywood, *Internal Combustion Engine Fundamentals*, 2.ª ed., McGraw-Hill 2018 — [editorial](https://www.mheducation.com/highered/mhp/product/internal-combustion-engine-fundamentals-2e.html) | De pago | Recambio de gases, C_d, combustión, modelado; recomendado por Ange |
| Toro, *Riemann Solvers and Numerical Methods for Fluid Dynamics*, 3.ª ed., Springer 2009 — [DOI 10.1007/b79761](https://doi.org/10.1007/b79761) | De pago | Godunov, HLL, HLLC, Roe, MUSCL, TVD, CFL: el núcleo del solucionador 1D |
| Winterbone & Pearson, *Theory of Engine Manifold Design*, PEP 2000 (reseña [DOI 10.1115/1.1421124](https://doi.org/10.1115/1.1421124)); *Design Techniques for Engine Manifolds*, 1999, ISBN 9781860581793 | De pago | MOC frente a LW/TVD y condiciones de contorno en colectores |
| Onorati & Montenegro, *1D and Multi-D Modeling Techniques for IC Engine Simulation*, SAE 2020 — [DOI 10.4271/9780768099522](https://doi.org/10.4271/9780768099522); 2.ª ed. [DOI 10.4271/9781468609943](https://doi.org/10.4271/9781468609943) | De pago | Sucesor moderno de Winterbone & Pearson, escrito por los autores de GASDYN |
| Blair, *Design and Simulation of Four-Stroke Engines*, SAE R-186, 1999 — [DOI 10.4271/r-186](https://doi.org/10.4271/r-186); *Two-Stroke*, R-161 — [DOI 10.4271/r-161](https://doi.org/10.4271/r-161) | De pago | Código 1D no isentrópico validado, geometría de válvulas y levas, barrido en dos tiempos |
| Benson, *The Thermodynamics and Gas Dynamics of Internal-Combustion Engines*, Vol. I, OUP 1982 — [reseña JFM](https://www.cambridge.org/core/journals/journal-of-fluid-mechanics/article/abs/thermodynamics-and-gas-dynamics-of-inernal-combustion-engines-volume-i-by-r-s-benson-edited-by-j-h-horlock-and-d-e-winterbone-oxford-university-press-1982-573-pp-5500/ED217F309DB52AB3CA74010E7FF68D4F) | De pago | Clásico del MOC; interés histórico |
| Guzzella & Onder, *Introduction to Modeling and Control of IC Engine Systems*, 2.ª ed., Springer 2010 — [DOI](https://doi.org/10.1007/978-3-642-10775-7) | De pago | Receptores, orificio y mariposa, MVEM, regularización cerca de Π→1 |
| Eriksson & Nielsen, *Modeling and Control of Engines and Drivelines*, Wiley 2014 — [DOI](https://doi.org/10.1002/9781118536186); Kiencke & Nielsen, Springer 2005 — [DOI](https://doi.org/10.1007/b137654) | De pago | Transmisión, embrague, vehículo |
| Kirkpatrick, *Internal Combustion Engines: Applied Thermosciences*, 4.ª ed., Wiley 2020 — [DOI](https://doi.org/10.1002/9781119454564) | De pago (código abierto) | El libro de los MATLAB de Colorado State |
| Stone, *Introduction to IC Engines*, 4.ª ed. — [DOI](https://doi.org/10.1007/978-1-137-02829-7); Ghojel, *Fundamentals of Heat Engines*, 2020 — [DOI](https://doi.org/10.1002/9781119548829) | De pago | Alternativas a Heywood |
| Munjal, *Acoustics of Ducts and Mufflers*, 2.ª ed., Wiley, ISBN 978-1-118-44312-5 — [Wiley-VCH](https://www.wiley-vch.de/de/fachgebiete/ingenieurwesen/acoustics-of-ducts-and-mufflers-978-1-118-44312-5) | De pago | Matrices de transferencia para silenciadores |
| Smith, *Physical Audio Signal Processing* — [PASP](https://ccrma.stanford.edu/~jos/pasp/); [Digital Filters](https://ccrma.stanford.edu/~jos/filters/); [SASP](https://ccrma.stanford.edu/~jos/sasp/) | Abierto | Guías de onda, Kelly-Lochbaum, retardo fraccional, filtros de onda digital (WDF), convolución |
| Farnell, *Designing Sound*, MIT Press 2010 — [Práctica 22 "Cars"](https://mitp-content-server.mit.edu/books/content/sectbyfn/books_pres_0/8375/designing_sound.zip/practical22.html) | Libro de pago, ejemplos abiertos | Motor procedural en Pure Data |
| MIT OCW 2.61 Internal Combustion Engines — [apuntes](https://ocw.mit.edu/courses/2-61-internal-combustion-engines-spring-2017/resources/lecture-notes/) | Abierto (CC BY-NC-SA) | Simulación de ciclo, admisión y escape, detonación, calor, fricción, turbo |
| Fabian, *Data-Oriented Design* — [libro online](https://www.dataorienteddesign.com/dodbook/) | Abierto | Organización SoA de los datos |

### Dinámica de gases 1D y esquemas numéricos

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Benson, Garg & Woollatt 1964, *Int. J. Mech. Sci.* 6(1) — [DOI](https://doi.org/10.1016/0020-7403(64)90009-8) | De pago | Origen del MOC numérico en motores |
| Onorati, Winterbone & Pearson, SAE 930428 — [DOI](https://doi.org/10.4271/930428) | De pago | LW2+FCT frente a MOC: el esquema cambia el contenido de alta frecuencia (y por tanto el sonido) |
| Pearson & Winterbone, SAE 900676 — [DOI](https://doi.org/10.4271/900676); Winterbone & Yoshitomi, SAE 900677 — [DOI](https://doi.org/10.4271/900677) | De pago | Simulación rápida de ondas en la admisión y su precisión |
| Liu, Schorn, Schernus & Peng, SAE 960078 — [DOI](https://doi.org/10.4271/960078) | De pago | Caso del tubo cónico: problemas de conservación en todos los esquemas |
| Galindo, Serrano, Arnau & Piqueras, *JEGTP* 131(3) 2009 — [DOI](https://doi.org/10.1115/1.2983015); ICES2008-1610 — [DOI](https://doi.org/10.1115/ICES2008-1610); *Math. Comput. Model.* 2009 — [DOI](https://doi.org/10.1016/j.mcm.2009.05.011) | De pago | Paso temporal independiente por elemento: el cuello de botella CFL |
| Montenegro, Onorati, Piscaglia & D'Errico, SAE 2007-01-0495 — [DOI](https://doi.org/10.4271/2007-01-0495) | De pago | HLLC en producción (GASDYN), V10 con 5→1, Helmholtz |
| Toro, Spruce & Speares 1994 — [DOI](https://doi.org/10.1007/BF01414629); Toro 2019 — [DOI](https://doi.org/10.1007/s00193-019-00912-4) | De pago | Artículos originales de HLLC |
| Keblawi & McPhee, SAE 2019-01-0190 — [DOI](https://doi.org/10.4271/2019-01-0190) | De pago | MOC por colocación ortogonal: pocos nodos, tiempo real |
| Hoang et al., LTS explícito de alto orden — [PDF](https://people.math.sc.edu/wangzhu/reference/Hoang2020MCOM190522-LTS_CLs.pdf); Tan et al., *JCP* 200, 2004 — [PDF](https://www.math.hkbu.edu.hk/~ttang/Papers/localstep.pdf) | Abierto (enlace del primero inestable) | Paso local con corrección de flujos que conserva la masa |
| Upwind CE-SE — [arXiv:2410.22923](https://arxiv.org/pdf/2410.22923) | Abierto | Análisis genérico del esquema CE-SE (opción de OpenWAM) |
| Blair & Magee, SAE 932399 — [DOI](https://doi.org/10.4271/932399); Blair, SAE 940395 — [DOI](https://doi.org/10.4271/940395); Fleck et al., SAE 982055 — [DOI](https://doi.org/10.4271/982055) | De pago | Cambios de sección y uniones no isentrópicas con pérdidas |
| Kirkpatrick et al., SAE 941685 — [DOI](https://doi.org/10.4271/941685); Blair et al., SAE 950276 — [DOI](https://doi.org/10.4271/950276) | De pago | Datos experimentales para validar códigos 1D |
| SAE 2015-24-2399 — [SAE](https://saemobilus.sae.org/content/2015-24-2399); SAE 2013-24-0069 — [SAE](https://saemobilus.sae.org/papers/improved-multi-pipe-junction-model-engine-thermodynamic-gas-dynamic-simulations-2013-24-0069) | De pago | Modelos de unión más recientes (autores sin verificar) |

### Modelos 0D, valor medio y tiempo real industrial

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Hendricks & Sorenson, SAE 900616 — [DOI](https://doi.org/10.4271/900616); SAE 910258 — [DOI](https://doi.org/10.4271/910258); Müller et al., SAE 980784 — [DOI](https://doi.org/10.4271/980784) | De pago | MVEM de 3 estados con ±2 %: nivel de respaldo y comprobación del caudal medio |
| Hendricks, SAE 890564 — [DOI](https://doi.org/10.4271/890564); Jensen et al., SAE 910070 — [DOI](https://doi.org/10.4271/910070); Moskwa & Hedrick 1992 — [DOI](https://doi.org/10.1115/1.2896525) | De pago | MVEM diésel y para control |
| Cipollone & Sciarretta, SAE 2001-01-0579 — [DOI](https://doi.org/10.4271/2001-01-0579); versión de 1999 — [DOI](https://doi.org/10.1142/9789814447317_0032) | De pago | Modelo cuasi-propagatorio: el nivel más barato que todavía da sintonía por ondas |
| Ogata et al. (Honda), JSAE 20239001 — [JSAE](https://tech.jsae.or.jp/paperinfo/en/content/conf2023-02.01/) | De pago | QPM en HILS de tiempo real |
| Pacitti et al., SAE 2008-01-1006 — [DOI](https://doi.org/10.4271/2008-01-1006); Wurzenberger et al., SAE 2009-01-0589 — [DOI](https://doi.org/10.4271/2009-01-0589); Xia et al., SAE 2018-01-1245 — [DOI](https://doi.org/10.4271/2018-01-1245); Andric et al., SAE 2018-01-0874 — [DOI](https://doi.org/10.4271/2018-01-0874); Papadimitriou et al., SAE 2008-01-0990 — [DOI](https://doi.org/10.4271/2008-01-0990) | De pago | Arquitectura industrial: cilindros resueltos por ángulo con tubos simplificados |
| Stockar et al., SAE 2016 — [Penn State](https://pure.psu.edu/en/publications/fast-simulation-of-wave-action-in-engine-air-path-systems-using-m/) | De pago | Reducción de orden de modelo para la acción de ondas |
| Hribernik & Bombek 1995 — [SV-JME](https://www.sv-jme.eu/article/comparison-of-zero-and-one-dimensional-methods-for-simulation-of-the-process-in-turbocharged-diesel-engines/) | Abierto (resumen) | Comparación 0D frente a 1D (sin cifras en el resumen) |
| Realis WAVE-RT — [Speedgoat](https://www.speedgoat.com/products-services/third-party-connections/wave-rt-realis-simulation) | Comercial | Solucionador de tiempo real cuya fidelidad "approaches full 1D CFD" |

### Combustión

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Ghojel, *IJER* 11(4) 2010 — [DOI](https://doi.org/10.1243/14680874JER06510) | De pago | Revisión de la función de Wiebe |
| Hellstrom, Stefanopoulou & Jiang, ICEF2013-19068 — [PDF](https://public.websites.umich.edu/~annastef/papers_hcci/HellstromASME2013ICEFall.pdf) | Abierto | Elección de *a*, ajuste de doble Wiebe |
| Watson et al., SAE 800029 — [DOI](https://doi.org/10.4271/800029); Miyamoto et al., SAE 850107 — [DOI](https://doi.org/10.4271/850107); Chmela & Orthaber, SAE 1999-01-0186 — [DOI](https://doi.org/10.4271/1999-01-0186) | De pago | Diésel: doble Wiebe y combustión controlada por mezcla |
| Blizard & Keck, SAE 740191 — [DOI](https://doi.org/10.4271/740191); Tabaczynski et al., SAE 770647 — [DOI](https://doi.org/10.4271/770647); Keck 1982 — [DOI](https://doi.org/10.1016/s0082-0784(82)80322-6); Beretta et al. 1983 — [DOI](https://doi.org/10.1016/0010-2180(83)90135-9) | De pago | Modelo cuasi-dimensional de arrastre turbulento |
| Metghalchi & Keck 1982 — [DOI](https://doi.org/10.1016/0010-2180(82)90127-4) | De pago | Velocidad laminar (las constantes que usa engine-sim) |
| Gatowski et al., SAE 841359 — [DOI](https://doi.org/10.4271/841359); Rassweiler & Withrow, SAE 380139 — [DOI](https://doi.org/10.4271/380139) | De pago | Análisis de liberación de calor a partir de curvas de presión (validación) |
| Eriksson & Andersson, SAE 2002-01-0371 — [DOI](https://doi.org/10.4271/2002-01-0371) | De pago | Presión de cilindro analítica, más barata que integrar EDO |
| Livengood & Wu 1955 — [DOI](https://doi.org/10.1016/S0082-0784(55)80047-1); Douaud & Eyzat, SAE 780080 — [DOI](https://doi.org/10.4271/780080); [UDF de Fluent](https://www.afs.enea.it/project/neptunius/docs/fluent/html/udf/node40.htm) | De pago / abierto | Detonación barata (unidades de τ por verificar) |

### Transferencia de calor y propiedades del gas

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Woschni, SAE 670931 — [DOI](https://doi.org/10.4271/670931) | De pago | Coeficiente de transferencia de calor por defecto |
| Trung 2021, *Arch. Thermodyn.* 42(3) — [DOI](https://doi.org/10.24425/ather.2021.138114) | Abierto | Constantes de Woschni (con corrección de swirl) y de Hohenberg |
| Hohenberg, SAE 790825 — [DOI](https://doi.org/10.4271/790825); Annand 1963 — [DOI](https://doi.org/10.1243/PIME_PROC_1963_177_069_02) | De pago | Alternativas a Woschni |
| Depcik & Assanis, SAE 2002-01-0372 — [DOI](https://doi.org/10.4271/2002-01-0372) | De pago | Calor en lumbreras de admisión y escape |
| [libICEpost Woschni](https://libice-post.readthedocs.io/en/latest/autoapi/libICEpost/src/engineModel/HeatTransferModel/Woschni/index.html); [apuntes de Kirkpatrick](https://www.engr.colostate.edu/~allan/heat_trans/page7a/page7a.html) | Abierto | Implementaciones de referencia |
| McBride, Zehe & Gordon, NASA/TP-2002-211556 — [NTRS](https://ntrs.nasa.gov/citations/20020085330); [Modelica.Media](https://doc.modelica.org/om/Modelica.Media.IdealGases.Common.DataRecord.html) | Abierto | Polinomios c_p(T) para más de 2.000 especies |
| Grill et al., SAE 2007-01-0936 — [DOI](https://doi.org/10.4271/2007-01-0936) | De pago | Cálculo rápido de propiedades de mezcla |

### Válvulas, fricción, mecánica y extensiones

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Kastner et al., Proc. IMechE 1963 — [DOI](https://doi.org/10.1243/PIME_PROC_1963_178_063_02) | De pago | C_d de válvula de asiento frente a alzada |
| Soriano et al., ENCIT 2012 — [PDF](https://www.abcm.org.br/anais/encit/2012/links/pdf/ENCIT2012-158.pdf) | Abierto | Coeficiente de descarga estacionario, área de flujo |
| Chen & Flynn, SAE 650733 — [DOI](https://doi.org/10.4271/650733); Pipitone, SAE 2009-01-1984 — [DOI](https://doi.org/10.4271/2009-01-1984) | De pago | Fricción simple y sus límites |
| Patton, Nitschke & Heywood, SAE 890836 — [DOI](https://doi.org/10.4271/890836); Sandoval & Heywood, SAE 2003-01-0725 — [DOI](https://doi.org/10.4271/2003-01-0725) | De pago | Fricción por componentes |
| Sandoval, tesis B.S. MIT 2002 — [PDF](https://dspace.mit.edu/bitstream/1721.1/80657/2/52940875-MIT.pdf) | Abierto | Ecuaciones completas de fricción por componentes (verificar el texto extraído) |
| Rizzoni, IEEE TVT 1989 — [DOI](https://doi.org/10.1109/25.45470); Brand et al., SAE 2005-01-0228 — [DOI](https://doi.org/10.4271/2005-01-0228); Kimura et al., SAE 2001-01-2719 — [DOI](https://doi.org/10.4271/2001-01-2719) | De pago | Dinámica del cigüeñal y torsión |
| Moraal & Kolmanovsky, SAE 1999-01-0908 — [DOI](https://doi.org/10.4271/1999-01-0908); Watson & Janota 1982 — [capítulo](https://doi.org/10.1007/978-1-349-04024-7_6) | De pago | Turbo: mapas y extrapolación |
| Dos tiempos: Krieger et al., SAE 690135 — [DOI](https://doi.org/10.4271/690135); Bozza & Gimelli, SAE 2004-01-0999 — [DOI](https://doi.org/10.4271/2004-01-0999). Wankel: Danieli et al., SAE 740186 — [DOI](https://doi.org/10.4271/740186); SAE 780416 — [DOI](https://doi.org/10.4271/780416). Diésel: Assanis & Heywood, SAE 860329 — [DOI](https://doi.org/10.4271/860329) | De pago | Extensiones futuras |

### Acústica de conductos, silenciadores y radiación

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Levine & Schwinger, *Phys. Rev.* 73, 1948 — [DOI](https://doi.org/10.1103/PhysRev.73.383) | De pago | Reflexión y corrección de extremo de un tubo sin brida |
| Silva et al., aproximaciones de la impedancia de radiación — [arXiv:0811.3625](https://ar5iv.labs.arxiv.org/html/0811.3625) | Abierto | Fórmulas cerradas para el filtro del extremo abierto |
| Payri, Desantes & Torregrosa, *JSV* 188(1) 1995 — [DOI](https://doi.org/10.1006/jsvi.1995.0580) | De pago | Condición acústica lineal dentro de un cálculo 1D no lineal |
| Torregrosa, Broatch & Payri, SAE 1999-01-1650 — [DOI](https://doi.org/10.4271/1999-01-1650); SAE 2000-01-0728 — [DOI](https://doi.org/10.4271/2000-01-0728) | De pago | Geometría del colector frente a ruido; TMM en silenciadores |
| Macián, Torregrosa, Broatch et al., resumen UPV 2011 — [PDF](https://imm.webs.upv.es/jornadas/2011/Abstracts/TorregrosaCMT.pdf) | Abierto | Identificación de la fuente (impedancia y fuerza) desde simulación 1D |
| Onorati et al., SAE 2006-01-1541 — [DOI](https://doi.org/10.4271/2006-01-1541); Montenegro et al., *Comput. Fluids* 71, 2013 — [DOI](https://doi.org/10.1016/j.compfluid.2012.10.016); Della Torre et al. 2017 — [DOI](https://doi.org/10.1007/978-3-658-16988-6_111) | De pago | Silenciadores con 1D, 1D–3D y cuasi-3D |
| Montenegro, Della Torre, Onorati & Fairbrother, *Adv. Acoust. Vib.* 2013 — [DOI](https://doi.org/10.1155/2013/546120) | Abierto | Red cuasi-3D de celdas 0D para silenciadores con perforados y absorbente: barata y explícita |
| Selamet et al., SAE 940612 — [DOI](https://doi.org/10.4271/940612); JASA 1995 — [DOI](https://doi.org/10.1121/1.414224); JASA 1997 — [DOI](https://doi.org/10.1121/1.417986) | De pago | Resonadores de Helmholtz |
| Mackey, Blair & Fleck, SAE 961806 — [DOI](https://doi.org/10.4271/961806) | De pago | Ruido simulado frente a medido desde 1D |
| Munjal, *Sadhana* 1990 — [DOI](https://doi.org/10.1007/BF02760468) | De pago | Revisión de acústica de conductos con flujo |
| Gamma Technologies, acústica de admisión y escape — [web](https://gtisoft.com/intake-and-exhaust-acoustics/) | Abierto (comercial) | Práctica industrial: ruido de orificio desde 1D, con salida de audio |

### Síntesis de sonido de motor

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Baldan, Lachambre, Delle Monache & Boussard, IEEE SIVE 2015 — [DOI](https://doi.org/10.1109/SIVE.2015.7361287); [postprint](https://air.iuav.it/retrieve/de164c2a-5461-60ee-e053-3a05fe0a7787/SIVE15_submission_4.pdf) | Abierto (postprint) | Motor con guías de onda: cilindros, colectores, silenciador, petardeos |
| Amman & Das, IEEE TIE 2001 — [DOI](https://doi.org/10.1109/41.904583); Cascone et al., US 6.959.094 — [USPTO](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/6959094) | De pago / abierto | Análisis y resíntesis; arte previo con base física |
| Jagla, Maillard & Martin, JASA 2012 — [DOI](https://doi.org/10.1121/1.4754663); ICASSP 2012, DOI 10.1109/ICASSP.2012.6287894 | De pago | PSOLA para motores (granular) |
| Cao et al., SAE 2020 — [DOI](https://doi.org/10.4271/10-04-02-0011); Li et al. 2024 — [DOI](https://doi.org/10.1007/978-981-97-0601-3_15) | De pago | Síntesis por órdenes y red neuronal en tiempo real |
| Engel et al., DDSP, ICLR 2020 — [arXiv:2001.04643](https://arxiv.org/abs/2001.04643); Ganis et al. 2021 — [arXiv:2103.07220](https://arxiv.org/abs/2103.07220) | Abierto | Marco DDSP y su ejecución en tiempo real |
| Lobato et al., Motor2Synth, AES 2025 — [resumen](https://cdn.head-acoustics.com/fileadmin/data/global/Abstracts/Abstract-AES-2025-Motor2Synth-Leveraging-DDSP-for-Generating-ASD-Compatible-Combustion-Engine-Sounds.pdf) | Resumen abierto / de pago | Los detectores de tono musicales fallan con motores: estimador de RPM propio |
| Doerfler & Wyse 2026, PTR — [arXiv:2603.09391](https://arxiv.org/abs/2603.09391); dataset — [arXiv:2603.07584](https://arxiv.org/abs/2603.07584); tesis de Doerfler 2025 — [RECERCAT](https://recercat.cat/handle/10230/72480) | Abierto (CC BY-NC-SA / NC) | Pulsos por orden de encendido, velocidad del sonido térmica, resonadores; 19 h de audio con RPM y par |
| Caviezel 2026, BOOM Library — [blog](https://www.boomlibrary.com/blog/the-car-engine-sound-primer-mike-caviezel/); Designing Sound 2014 — [artículo](https://designingsound.org/2014/08/11/vehicle-engine-design-project-cars-forza-motorsport-5-and-rev/); BeamNG — [docs](https://documentation.beamng.com/modding/vehicle/sections/sounds/engine_audio/); AudioMotors — [MCV](https://www.mcvuk.com/development/how-to-make-racing-car-engines-roar-using-audiomotors-fmod) | Abierto | Práctica de la industria del videojuego: qué debe cumplir tu sonido (RPM exactas, carga y retención, perspectivas) |
| Hilo de GameDev.net sobre audio procedural de motor — [foro](https://gamedev.net/forums/topic/713424-procedural-engine-audio/) | Abierto | Errores habituales: la posición del pistón da DC; la velocidad satura a altas RPM |

### DSP: retardo fraccional, remuestreo y convolución

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Laakso et al., "Splitting the unit delay", IEEE SPM 1996 — [DOI](https://doi.org/10.1109/79.482137) | De pago | Tutorial de Lagrange y Thiran |
| Smith, Digital Audio Resampling — [web](https://ccrma.stanford.edu/~jos/resample/); DC Blocker — [web](https://ccrma.stanford.edu/~jos/filters/DC_Blocker.html) | Abierto | Remuestreo limitado en banda y filtro de DC |
| Gardner, "Efficient Convolution without Input-Output Delay", JAES 1995 — [AES](https://aes2.org/publications/elibrary-page/?id=7957) | De pago | Convolución particionada no uniforme sin latencia |
| Wefers & Vorländer, DAFx-11 — [PDF](https://www.dafx.de/paper-archive/2011/Papers/74_e.pdf); Wefers, tesis RWTH 2015 — [DOI](https://doi.org/10.18154/RWTH-2015-02185) | Abierto / sin confirmar | Particiones óptimas de convolución FFT uniforme |

### Fluidos en gráficos y aeroacústica (contexto: por qué no sirven en tiempo de ejecución)

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Stam, Stable Fluids 1999 — [DOI](https://doi.org/10.1145/311535.311548); GDC 2003 — [PDF](https://www.dgp.toronto.edu/public_user/stam/reality/Research/pdf/GDC03.pdf) | De pago / abierto | Contexto: incompresible y difusivo |
| Bridson, *Fluid Simulation for Computer Graphics* — [web](https://www.cs.ubc.ca/~rbridson/fluidsimulation/); Müller et al., SCA 2003 — [PDF](https://matthias-research.github.io/pages/publications/sca03.pdf) | De pago / abierto | Estructuras de datos de fluidos; SPH |
| Dobashi et al., TOG 2003 — [DOI](https://doi.org/10.1145/882262.882339); CGF 2004 — [DOI](https://doi.org/10.1111/j.1467-8659.2004.00785.x) | De pago | Texturas de sonido precalculadas; idea para ruido de flujo horneado |
| Wang, Qu, Langlois & James, TOG 2018 — [DOI](https://doi.org/10.1145/3197517.3201318); Zheng & James 2009 — [DOI](https://doi.org/10.1145/1531326.1531343); Moss et al. 2010 — [DOI](https://doi.org/10.1145/1805964.1805965) | De pago | Sonido basado en ondas, offline |
| Liu & Manocha, revisión — [arXiv:2011.05538](https://arxiv.org/abs/2011.05538); Shao & Li, LBM en aeroacústica 2019 — [DOI](https://doi.org/10.24425/aoa.2019.128486) | Abierto | Estado del arte en síntesis y propagación; LBM |

### Rendimiento y simulación en tiempo real

| Referencia | Acceso | Para qué te sirve |
|---|---|---|
| Fiedler, "Fix Your Timestep!" — [Gaffer On Games](https://gafferongames.com/post/fix_your_timestep/) | Abierto | Acumulador, espiral de la muerte, determinismo |
| Macklin et al., XPBD 2016 — [PDF](https://matthias-research.github.io/pages/publications/XPBD.pdf); "Small Steps", SCA 2019 — [PDF](https://matthias-research.github.io/pages/publications/smallsteps.pdf); [Ten Minute Physics](https://matthias-research.github.io/pages/tenMinutePhysics/index.html) | Abierto | Muchos subpasos baratos frente a pocos pasos grandes |
| Catto, Solver2D 2024 — [blog](https://box2d.org/posts/2024/02/solver2d/); GDC 2006 — [PDF](https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf) | Abierto | Solucionadores de Gauss-Seidel y "soft step" |
| Bencina, "Real-time audio programming 101" — [web](http://www.rossbencina.com/code/real-time-audio-programming-101-time-waits-for-nothing) | Abierto | Reglas del callback de audio |
| Rust Performance Book — [build](https://nnethercote.github.io/perf-book/build-configuration.html), [profiling](https://nnethercote.github.io/perf-book/profiling.html) | Abierto | LTO, PGO, perfiladores |
| Tytel, "Unexpected Optimization #1" — [blog](https://tytel.org/blog/2015/02/10/unexpected-optimization-01/) | Abierto | Subnormales en DSP |

## Lagunas y elementos sin verificar

Los huecos más importantes son de rendimiento: **no se encontró ninguna medida publicada de CPU** de un solucionador 1D de gases de motor a frecuencia de audio. Tampoco hay datos del coste de engine-sim según su frecuencia de simulación, ni cifras de CPU de los modelos procedurales o neuronales. Las estimaciones del informe (20–30 µs de paso CFL, 10⁷ celdas/s, 1–2 GFLOP/s, 4 kHz de corte transversal) son cálculos propios de las notas, no datos publicados. Ningún resumen accesible de los modelos industriales de tiempo real (WAVE-RT, AVL, GT-SUITE FRM, Honda QPM) indica Δt, Δx ni carga de CPU. Los textos completos están en SAE Mobilus, de pago, igual que casi todos los artículos SAE, IMechE y Elsevier citados. Las notas no comprobaron si existen copias de autor en ResearchGate. Los repositorios institucionales RiuNet (UPV), IRIS (PoliMi) y HAL estaban tras comprobaciones anti-bot, así que no se confirmó qué tesis del CMT o de PoliMi tienen PDF abierto.

Sobre AngeTheGreat: **no se vieron los vídeos** (no hay transcripciones), y su código nuevo es cerrado. Por tanto, el esquema numérico del nuevo Engine Simulator solo se conoce por los títulos de los capítulos. No se pudieron leer las issues ni las discusiones de GitHub. La búsqueda de ports de engine-sim a Rust, WASM, Godot o Unity no fue exhaustiva porque la API de búsqueda de GitHub no estaba disponible. La atribución de la velocidad laminar de engine-sim a Metghalchi & Keck es una deducción. El artículo de Starlog contradice el código en puntos clave, y su cifra de un error de potencia del 15–20 % no tiene fuente.

En termodinámica, hay varios puntos que conviene comprobar antes de programar:

- Las constantes por defecto de Chen–Flynn no tienen fuente primaria accesible.
- La extracción del texto de la tesis de Sandoval corrompe algunos términos.
- Las unidades de τ en Douaud–Eyzat no están confirmadas (¿17,68 ms o 0,01768 s?).
- El exponente de T en Woschni difiere entre implementaciones (−0,53 frente a −0,55).
- Las ecuaciones exactas de Blizard–Keck y Tabaczynski, los C_d de Kastner y el apéndice de Heywood con la ecuación de orificio no se consultaron en la fuente original. Las formas que se dan en el informe son las estándar.
- No se verificó la numeración de capítulos de la 2.ª edición de Heywood.

En software, el aviso de xrun de `cpal` figura en "Unreleased" y puede no estar en la 0.18.2. No se revisaron el crate `multiversion` ni `nih_plug`. La licencia del Sound Design Toolkit y los autores de SAE 2019-24-0174, 970836, 2015-24-2399 y 2013-24-0069 quedaron sin verificar.

## Conclusión

Tu proyecto ya no consiste en investigar si se puede, sino en cuánta física decides pagar. La lectura del código cambia la imagen popular de engine-sim. Su "física que suena" es sobre todo **temporización de pulsos 0D más un DSP muy afinado**: paso bajo a 1,9 kHz, IR, ruido, jitter y AGC. Eso significa que un Rust bien hecho puede igualarlo con muy poco coste, y además corregirle fallos y bloqueos. El rasgo diferencial que el propio Ange persiguió después, la sintonía real de escapes por ondas, exige el salto a volúmenes finitos 1D. Ahí el riesgo técnico no está en la física, que está bien documentada desde los años 90, sino en que la disipación del esquema y la condición del extremo abierto **se oyen**, y nadie ha publicado cómo medir eso en tiempo real.

La estrategia más sólida es una escala de calidad explícita, de modo que el mismo motor pueda correr como 0D más guías de onda en un portátil viejo y como 1D HLLC en un ordenador de sobremesa. Esa escala debe estar gobernada por el llenado del búfer de audio, no por una opción en un menú. Así el requisito de "máquinas lentas" pasa de ser una restricción del diseño a ser una característica del producto. También deja el camino abierto a validar contra datos publicados, que es justo lo que engine-sim nunca pretendió hacer. Hay además una ventaja local: el grupo más prolífico en dinámica de gases 1D y ruido de escape de esta bibliografía, el CMT-Motores Térmicos de la UPV, está en Valencia, y el código de OpenWAM está escrito en buena parte con identificadores en español.

Da quel momento, chi clona la repo dovrà fare:

git clone --recursive <repo>

oppure, se ha già clonato:

git submodule update --init --recursive

altrimenti src/husky_navigation risulterà vuota o non inizializzata.



Un'altra cosa importante: quando modifichi il codice dentro src/husky_navigation, dovrai fare:

cd src/husky_navigation
git add .
git commit -m "..."
git push

e poi tornare nella repo principale e aggiornare il puntatore del submodule:

cd ../..
git add src/husky_navigation
git commit -m "Update husky_navigation submodule"
git push

Questo è il comportamento normale dei submodule e spesso è il punto che sorprende di più chi li usa per la prima volta.






























# Report modifiche — Allineamento alla repo ufficiale NaVILA

**Data:** 09/06/2026
**Obiettivo:** rendere il pipeline ROS 2 (NaVILA → Husky) il più fedele possibile all'inferenza
della repo ufficiale `AnjieCheng/NaVILA` (`evaluation/vlnce_baselines/navila_trainer.py`),
mantenendo però l'esecuzione su robot reale con controllo continuo e safety layer.

Riferimento ufficiale: il modello è una policy **step-sincrona** (cattura frame → inferenza →
esegue UNA primitiva di moto → cattura frame → ...), con quantizzazione delle azioni e history
spaziata per primitiva. Checkpoint usato in inferenza: `a8cheng/navila-llama3-8b-8f`.

---

## 1. Nodo NaVILA (`navila_node.py`) — il cuore della fedeltà

### Caricamento e inferenza del modello
- Allineato `model.generate(...)` all'ufficiale: tensore immagini passato diretto
  (`images=image_tensor`, non in lista), `do_sample=False`, `num_beams=1`,
  `max_new_tokens=32`, `use_cache=True`, stesso `KeywordsStoppingCriteria`.
- Prompt identico all'ufficiale: template `llama_3`, testo "Imagine you are a robot programmed
  for navigation tasks..." con `<image>` per gli N-1 frame storici + 1 osservazione corrente.
- `process_images` + `tokenizer_image_token` + `IMAGE_TOKEN_INDEX` invariati rispetto all'ufficiale.

### Parsing dell'output → con magnitudine (prima veniva scartata)
- Sostituito il parser a punteggio pesato (`_ACTION_PATTERNS`) e il classificatore Phi-3 con la
  logica ufficiale: 4 pattern (`stop`, `move forward`, `turn left`, `turn right`) + estrazione
  del valore numerico.
- `parse_navila_output` ora ritorna `(action, value, unit)` invece del solo token.
- Quantizzazione ufficiale: distanze sui multipli di 25 cm `{25,50,75}`, angoli sui multipli di
  15° `{15,30,45}`, applicata **solo se** il valore non è già multiplo. Default su parse fallito:
  forward 25 cm / 15°.
- **Rimosso del tutto Phi-3** (`Phi3Classifier`, `_PHI3_SYSTEM_PROMPT`) e il vocabolario esteso
  (`forward_fast`, `backward`, `curve_*`): non esistono nella repo, NaVILA non li produce.

### Campionamento + padding dello storico
- Riscritto `_sample_history` come replica fedele di `sample_and_pad_images` ufficiale:
  - padding in testa con **frame neri** (512×512) quando la storia è più corta di `num_video_frames`,
    invece di duplicare frame reali;
  - `num_frames-1` indici via `np.linspace(0, len-1, endpoint=False, dtype=int)` (troncamento),
    invece di `endpoint=True` + `round()`;
  - frame corrente sempre riservato all'ultima posizione.
- `max_history_frames` alzato (512) perché l'ufficiale non cappa la history: il `linspace` campiona
  su tutto l'episodio.

### Loop event-driven (era timer wall-clock a 2 Hz)
- Tolto il timer di inferenza come driver; ora il ciclo è **sincronizzato col moto**: una decisione →
  esecuzione → osservazione → decisione successiva.
- L'avanzamento avviene alla ricezione dello stato di completamento primitiva (vedi `action_node`),
  non a tempo di orologio.
- La history viene aggiornata **un frame per primitiva eseguita** (`_last_decision_frame` promosso
  al ciclo successivo), replicando `past_rgbs.append(...)` che nell'ufficiale gira a ogni step del loop.
- Timer residuo a 0.5 Hz mantenuto solo come poll di bootstrap idempotente (`_kick_drive`).

### Action queue (replica di `queue_actions`)
- Una decisione espande l'intera magnitudine in primitive: `value // 25` (forward) o `value // 15`
  (turn), con `max(1, ...)` per non azzerare su valori sotto soglia.
- Ne esegue 1 subito e accoda le restanti `(n-1)` in `self._queue`.
- Finché la coda non è vuota, **si replica la primitiva senza re-inferire** — esattamente come
  l'ufficiale consuma `queue_actions`. La re-inferenza riparte solo a coda esaurita.

### Gestione stato e robustezza
- Inizializzato `self._last_image_msg = None` (prima causava `AttributeError` al primo tick).
- Phi-3 (quando ancora presente) caricato da `model_path` locale invece che da `model_id`,
  per non sprecare il `snapshot_download` — poi rimosso del tutto.

---

## 2. Differenza voluta rispetto all'ufficiale (robot reale)

- L'ufficiale esegue la prima primitiva di una nuova decisione **nello stesso giro** (`envs.step`
  immediato in simulazione). Sul robot reale si pubblica il comando e si **attende il completamento
  fisico** prima di proseguire. La sequenza di azioni vista dal modello resta identica; cambia solo
  che ogni primitiva è gated dal moto reale invece che da uno step di simulazione.

---

## 3. Nodo Action (`action_node.py`) — esecuzione delle primitive

### Formato messaggi e vocabolario
- Parser dell'input riscritto per leggere `"<action> <value> <unit>"` (es. `forward 25 cm`,
  `turn_left 15 deg`, `stop`).
- Rimossi JSON, token nudi, `_action_map`, vocabolario esteso: tutto ciò che NaVILA non emette.

### Da open-loop a closed-loop su odometria
- Ogni primitiva esegue UN movimento misurato: si salva la posa iniziale da `/odom`, si calcola il
  progresso reale (distanza euclidea per forward, delta-yaw normalizzato per turn) e ci si ferma al
  raggiungimento del target (25 cm / 15°).
- Yaw ricavato dal quaternione senza dipendenze esterne.
- Margine anti-overshoot sul target (compensa la rampa di decelerazione).

### Segnalazione completamento vs abort (topic con payload)
- Pubblicato `/navila/primitive_status` (`std_msgs/String`) con payload `done` o `aborted`,
  che fa avanzare il loop del nodo NaVILA.
- `done`: completamento per misura odometrica → la coda prosegue, il frame entra nella history.
- `aborted`: completamento per **deadline failsafe** (robot bloccato / odom fermo) → il nodo NaVILA
  svuota la coda e **scarta** il frame della primitiva incompleta (movimento non avvenuto, fuori
  dalla distribuzione "un frame per primitiva completata").

### Failsafe e fix di robustezza
- Aggiunta deadline per primitiva (`3× tempo ideale + 1s`): se odom non avanza, abort invece di
  loop bloccato all'infinito.
- Rimosso il watchdog a tempo (incompatibile con le pause di inferenza tra primitive).
- Fix `TypeError` su `_deadline` None: guardia esplicita in `_publish_cb`, `_deadline` impostato
  prima di `_executing`, reset a `None` alla chiusura di ogni primitiva.
- Inizializzati `_executing`, `_start_pose`, `_prim_kind`, `_prim_target`, `_odom`, `_deadline`.

---

## 4. Nodo Safety (`safety_layer_node.py`)

- Confermata corretta la struttura (settorizzazione LiDAR, `sector_min` con mezzo FOV, fusione
  lidar+depth, fail-safe su timeout scan/cmd).
- **Da correggere:** zone di frenata invertite (`front_stop_dist` deve essere < `front_slow_dist`,
  altrimenti il rallentamento progressivo non si attiva e il denominatore dello scaling è negativo).
- Interazione col loop: quando il safety azzera il `cmd_vel`, il robot non avanza, la primitiva non
  si completa per misura e scatta l'`aborted` per deadline → NaVILA ridecide col nuovo frame.
- Da verificare l'encoding della depth ZED (metri vs millimetri) sul topic reale.

---

## 5. Punti aperti / da tarare nei test

- **Odometria che integra lo slittamento:** se il robot è bloccato ma le ruote girano, un'odometria
  a soli encoder fa crescere il `progress` e genera un `done` spurio. Usare la posa fusa con IMU
  (`/odometry/filtered` da `robot_localization`) e/o aggiungere un rilevamento di stallo
  (progresso che non avanza → abort).
- **Taratura:** velocità lineare/angolare, margine anti-overshoot, moltiplicatore della deadline
  failsafe.
- **Tooling:** `ros2 topic info -v` / `echo` falliscono con `unknown tag 'rclpy.type_hash.TypeHash'`
  per mismatch di versione `ros2cli` ↔ core nel container (non impatta il runtime). Workaround:
  `--qos-reliability reliable`; fix: riallineare i pacchetti `ros-humble-*`.

---

## Riepilogo: cosa rende il sistema fedele alla repo

| Aspetto | Ufficiale | Implementato |
|---|---|---|
| Generazione | greedy, 32 token, tensore diretto | identico |
| Prompt | template llama_3 + 7 storici + 1 corrente | identico |
| Parsing azioni | 4 pattern + magnitudine + quantizzazione | identico |
| Campionamento frame | sample_and_pad_images (nero, endpoint=False, int) | replica fedele |
| Loop | step-sincrono, 1 primitiva per step | event-driven gated dal moto reale |
| Coda azioni | queue_actions, replay senza re-inferire | replica fedele |
| History | 1 frame per primitiva eseguita | identico (gated da done/aborted) |
| Esecuzione fisica | envs.step istantaneo (sim) | closed-loop su odom + failsafe (reale) |
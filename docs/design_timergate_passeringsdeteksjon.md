# Timergate Passeringsdeteksjon - Funksjonell Designdokumentasjon v2.0

## Oversikt

Timergate-systemet implementerer presis passeringsdeteksjon ved å korrelere sensorbrudd fra infrarøde lysbarrierer på tvers av en målestolpe. Systemet bruker en **hybrid timing-arkitektur** for optimal presisjon og robusthet.

## Systemarkitektur

```
Målestolpe (ESP32)     ESP-NOW     Aksesspunkt (ESP32)     WebSocket     Klienter (Vue.js)
     ↓                    →               ↓                     →              ↓
  7 sensorer                      Passeringsdeteksjon               Timer-grensesnitt
     ↓                               ↓                                       ↓
K=1 meldinger                  Korrelasjon                              Brukervisning
                                    ↓
                              K=4 passeringer
```

## Målestolpe (timergate_pole.c)

### Sensor-teknologi
Målestolpen bruker **infrarød modulert lys** (9kHz PWM) for å redusere interferens fra sollys og andre lyskilder. Hver målestolpe har **7 sensorer** arrangert vertikalt.

### Sensordata og ESP-NOW
Målestolpen sender to typer meldinger via ESP-NOW:

#### K=0: ADC-verdier (kontinuerlig overvåking)
Sendes hver sekund for signalkvalitet og debugging.

#### K=1: Sensorendringer (hendelsesdrevet)
Sendes umiddelbart når en sensor endrer tilstand:

```c
typedef struct {
    uint8_t k;           // Meldingstype = 1
    uint32_t t;          // Timestamp sekunder
    uint32_t u;          // Timestamp mikrosekunder
    int32_t sensor_id;   // Sensor nummer 0-6
    int32_t break_state; // 1=brudd, 0=gjenopprettet
} __attribute__((packed)) esp_now_sensor_state_change_t;
```

**VIKTIG**: K=1 meldinger sendes ved **alle** sensorendringer, men kun `break_state=1` (brudd) behandles for passeringsdeteksjon. `break_state=0` (gjenopprettelse) brukes kun for logging og statistikk.

### ESP-NOW Datastruktur og Parsing

#### Målestolpe sender (timergate_pole.c):
```c
typedef struct {
    uint8_t k;           // Meldingstype = 1 (1 byte, posisjon 0)
    uint32_t t;          // Timestamp sekunder (4 bytes, posisjon 1-4)
    uint32_t u;          // Timestamp mikrosekunder (4 bytes, posisjon 5-8)
    int32_t sensor_id;   // Sensor nummer 0-6 (4 bytes, posisjon 9-12)
    int32_t break_state; // 1=brudd, 0=gjenopprettet (4 bytes, posisjon 13-16)
} __attribute__((packed)) esp_now_sensor_state_change_t;
```

#### Aksesspunkt mottar (timergate_server.c):
```c
// KORREKT parsing av ESP-NOW data array:
uint8_t k = data[0];  // Meldingstype

// Les sensor_id korrekt fra posisjon 9-12
int32_t sensor_id;
memcpy(&sensor_id, &data[9], sizeof(int32_t));

// Les break_state korrekt fra posisjon 13-16  
int32_t break_state;
memcpy(&break_state, &data[13], sizeof(int32_t));

// FORBEDRET: Presis logging av sensorendringer
ESP_LOGI(TAG, "🔄 SENSORENDRING MOTTATT: K=1, sensor_id=%d, break_state=%d (%s)", 
         sensor_id, break_state, (break_state == 1) ? "BRUDD" : "GJENOPPRETTET");

// KRITISK: Kun behandle faktiske brudd for passeringsdeteksjon
if (break_state == 1) {
    // Kall passeringsdeteksjon kun for brudd
    bool passage_detected = process_break_for_passage_detection(...);
} else {
    // Gjenopprettelse: Kun logging og statistikk
    ESP_LOGI(TAG, "✅ SENSOR GJENOPPRETTET - kun logging");
}
```

### Navnekonvensjoner og Terminologi

**VIKTIG**: Konsistent terminologi brukes i hele kodebasen:

| Konsept | Standardnavn | Tidligere navn | Beskrivelse |
|---------|-------------|----------------|-------------|
| Sensor tilstand | `break_state` | `break_value` | 1=brudd, 0=gjenopprettet |
| Sensor hendelse | K=1 sensorendring | K=1 sensorbrudd | Omfatter både brudd og gjenopprettelse |
| Sensor nummer | `sensor_id` | `sensor_id` | 0-6 (konsistent) |
| Tidsstempel sek | `t` | `t` | Unix timestamp sekunder |
| Tidsstempel μs | `u` | `u` | Mikrosekunder del |
| Meldingstype | `k` | `k` | 0=ADC, 1=endring, 2=config, 4=passering |

**Implementasjonsnote**: K=1 meldinger sendes ved **alle** sensorendringer, men kun `break_state=1` (brudd) behandles for passeringsdeteksjon. `break_state=0` (gjenopprettelse) brukes kun for logging og statistikk.

### Systemstatus
Målestolpen opererer med følgende tilstander:
- `STATUS_INITIALIZING`: Oppstart og WiFi-tilkobling
- `STATUS_CALIBRATING`: Automatisk kalibrering av sensorer
- `STATUS_READY`: Klar for passeringsdeteksjon
- `STATUS_ERROR`: Feilsituasjon (sensorer blokkert etc.)

### Kalibrering
Målestolpen har automatisk kalibreringssystem:
1. **Highpoint search**: Finn optimal PWM-styrke per sensor
2. **Offset-beregning**: Kalibrer for omgivelsesbetingelser  
3. **Break limit setting**: Sett terskelverdi basert på kalibrerte verdier
4. **Lagring**: Verdier lagres i NVS for persistent konfigurasjon

### Runtime-konfigurasjon
Aksesspunktet kan sende konfigurasjon til målestolper via ESP-NOW:

- **SET_BREAK** (CMD=0x04): Oppdater break_limit for spesifikk sensor
- **SET_ENABLED** (CMD=0x05): Aktivere/deaktivere sensorer
- **HSEARCH** (CMD=0x03): Trigger ny kalibrering

## Aksesspunkt (timergate_server.c)

### ESP-NOW Mottak og Prosessering
Aksesspunktet mottar ESP-NOW meldinger og prosesserer dem basert på meldingstype (K-verdi):

```c
// Prosessering av sensorendring-meldinger (K=1)
// VIKTIG: Kun break_state=1 (brudd) sendes til passeringsdeteksjon
static void process_break_for_passage_detection(
    const char* mac_str, 
    uint32_t timestamp_sec, 
    uint32_t timestamp_usec, 
    int32_t sensor_id, 
    int32_t break_state  // ← Kun kalt når break_state=1
)
```

## ⏰ KRITISK TIMING-DESIGN: Hybrid Timing-arkitektur

### Designbeslutning: Optimalisert Timing for Forskjellige Formål

**VIKTIG**: Timergate-systemet bruker **hybrid timing-arkitektur** som balanserer presisjon og konsistens for ulike timing-behov.

#### Begrunnelse:
1. **Forskjellige timing-behov**: Sequence-korrelasjoner krever konsistent tidsreferanse, mens passeringer krever autoritativ server-tid
2. **ESP-NOW latency varierer**: 1-10ms forsinkelse kan variere pga. systembelastning, men målestolpe-tid er konsistent for sequence-timing
3. **Optimal presisjon**: Hver timing-type bruker den mest presise tidskilden for sitt formål
4. **Praktisk funksjonalitet**: Sikrer at timeout-logikk fungerer korrekt med reelle sensor-intervaller

#### Implementering - Hybrid Timing:
```c
// Sequence-timing: Bruk målestolpe-tid for konsistente intervaller
uint64_t sensor_time_ms = (uint64_t)sensor_time_sec * 1000 + sensor_time_micros / 1000;
uint64_t sequence_start_ms = (uint64_t)passage_detectors[idx].first_break_time * 1000 + 
                            (uint64_t)passage_detectors[idx].first_break_micros / 1000;
uint64_t sequence_duration_ms = sensor_time_ms - sequence_start_ms;

// Passeringer og debounce: Bruk server-tid som autoritativ kilde
struct timeval tv;
gettimeofday(&tv, NULL);
uint32_t authoritative_time_sec = tv.tv_sec;
uint32_t authoritative_time_usec = tv.tv_usec;
```

#### Timing-hierarki og anvendelse:
- **Sequence-korrelasjoner**: Målestolpe-tid (konsistente intervaller for timeout-beregninger)
- **K4 WebSocket-meldinger**: Server-tid (autoritativ for klienter)
- **Debounce-logikk**: Server-tid (forhindrer spam på server-siden)
- **Logging og debugging**: Begge tider (for sammenligning og analyse)

### WebSocket Tidsstempel (K=4 meldinger)
Alle passeringer sendes til klienter med server-tid:
```json
{
    "M": "aa:bb:cc:dd:ee:ff",  // MAC-adresse til målestolpe
    "K": 4,                    // Meldingstype: Passering
    "T": 1672531200,           // Server timestamp sekunder ← CONFIRMATION TIME
    "U": 123456                // Server timestamp mikrosekunder ← CONFIRMATION TIME
}
```

**TIMING PHILOSOPHY**: Tidsstempelet representerer **confirmation time** - øyeblikket når `min_sensors` terskelen nås og passeringen bekreftes som gyldig. Dette er IKKE tidspunktet for første sensor i sekvensen, men når systemet faktisk kan garantere at en passering har skjedd.

**Eksempel**: Ved `min_sensors=3`:
- Sensor 1: 10:00:00.100 (registrert)
- Sensor 2: 10:00:00.150 (registrert) 
- Sensor 3: 10:00:00.200 (terskel nådd → **K4 sendes med T=10:00:00.200**)

**Begrunnelse for confirmation time**:
- Garanterer at tidsstempelet er fra en bekreftet passering
- Unngår tidsstempler fra falske positive første sensorer
- Gir konsistent timing uavhengig av hvor mange sensorer som til slutt utløses

## Passeringsdeteksjon - Fundamentale Parametere

Passeringsdeteksjon er kjernen i Timergate-systemet og styres av tre kritiske parametere som defineres i GUI-konfigurasjonen:

```json
{
  "debounce_ms": 1000,
  "min_sensors": 2,
  "timeout_ms": 500
}
```

### 1. DEBOUNCE_MS - Minimum tid mellom passeringer

**Funksjon**: Forhindrer "falske" passeringer
**Standard**: 1000ms (1 sekund)
**Bruksområde**: Sikrer at samme fysiske passering ikke registreres som flere separate passeringer

```c
// Faktisk implementering i aksesspunkt (bruker server-tid for debounce)
struct timeval current_server_time;
gettimeofday(&current_server_time, NULL);
uint64_t current_time_ms = (uint64_t)current_server_time.tv_sec * 1000 + current_server_time.tv_usec / 1000;
uint64_t last_passage_ms = (uint64_t)passage_detectors[detector_idx].last_passage_time * 1000 + 
                        passage_detectors[detector_idx].last_passage_micros / 1000;

if (passage_detectors[detector_idx].last_passage_time > 0) {
    uint64_t time_since_last_passage_ms = current_time_ms - last_passage_ms;
    if (time_since_last_passage_ms < passage_debounce_ms) {
        // Ignorer - for kort tid siden forrige passering
        return false;
    }
}
```

**Praktisk betydning**:
- Sikrer at samme person/objekt kun registreres én gang per passering
- Kritisk innstilling: må være lavere enn forventet løpstid, men høy nok til å forhindre falske triggere

### 2. MIN_SENSORS - Minimum antall sensorer som må utløses

**Funksjon**: Definerer hvor mange lysbarrierer som må brytes for gyldig passering
**Standard**: 2 sensorer
**Bruksområde**: Skelne mellom reelle passeringer og støy/delvis okklusion

```c
// Implementering i aksesspunkt
int broken_sensors = count_broken_sensors_in_timeframe(sensor_data, timeout_ms);
if (broken_sensors < min_sensors) {
    // Ikke nok sensorer utløst - ignorer
    return false;
}
```

**Praktisk betydning**:
- Forhindrer falske triggere fra insekter, blader, etc.
- Sikrer at hele objektet/personen passerer mållinjen
- Kritisk for å filtrere bort delvis okklusjon
- For lav verdi: Mange falske positiver
- For høy verdi: Kan miste gyldige passeringer

### 3. TIMEOUT_MS - Maksimal tid for sensorkorrelasjonsvindu

**Funksjon**: Definerer tidsvinduet hvor sensorbrudd skal grupperes som samme passering
**Standard**: 500ms
**Bruksområde**: Korrelere sensorbrudd fra samme fysiske passering

```c
// Faktisk implementering i aksesspunkt (bruker målestolpe-tid for sequence-timing)
uint64_t sensor_time_ms = (uint64_t)sensor_time_sec * 1000 + sensor_time_micros / 1000;
uint64_t sequence_start_ms = (uint64_t)passage_detectors[detector_idx].first_break_time * 1000 + 
                            (uint64_t)passage_detectors[detector_idx].first_break_micros / 1000;
uint64_t sequence_duration_ms = sensor_time_ms - sequence_start_ms;

if (sequence_duration_ms > sequence_timeout_ms) {
    // Ny passering startet
    evaluate_previous_passage();
    start_new_passage(sensor_id);
}
```

**Praktisk betydning**:
- Lar systemet "samle" sensorbrudd fra samme fysiske passering
- Bruker målestolpe-tid for konsistente intervall-beregninger
- Kompenserer for varierende ESP-NOW prosesseringstid på server
- Kritisk for korrekt telling av sensorer per passering
- For lav verdi: Splitter én passering i flere deler
- For høy verdi: Kan slå sammen separate passeringer

## 💾 Persistent Konfigurasjon (NVS)

### NVS-lagring av passeringsparametere
Alle passeringsdeteksjonsparametere lagres persistent i ESP32's Non-Volatile Storage (NVS) for å overleve restart og strømbrudd:

```c
// Funksjon for å lagre konfigurasjon til NVS
esp_err_t save_passage_config_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("timer_cfg", NVS_READWRITE, &nvs_handle);
    
    if (err == ESP_OK) {
        nvs_set_i32(nvs_handle, "debounce_ms", passage_debounce_ms);
        nvs_set_i32(nvs_handle, "min_sensors", min_sensors_for_passage);
        nvs_set_i32(nvs_handle, "timeout_ms", sequence_timeout_ms);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        return ESP_OK;
    }
    return err;
}
```

**Livssyklus for konfigurasjon**:
1. **Ved oppstart**: `load_passage_config_from_nvs()` kalles i `app_main()`
2. **Ved endring**: `save_passage_config_to_nvs()` kalles umiddelbart når GUI endrer verdier
3. **Ved factory reset**: NVS kan slettes og standardverdier gjenopprettes

**Fordeler med NVS**:
- ✅ Overlever restart og strømbrudd
- ✅ Overlever firmware-oppdateringer  
- ✅ Umiddelbar lagring ved endringer
- ✅ Fallback til standardverdier hvis NVS er tom

**NVS namespace og nøkler**:
- **Namespace**: `"timer_cfg"` (maks 15 karakterer)
- **Nøkler**: `"debounce_ms"`, `"min_sensors"`, `"timeout_ms"`

## Passeringsdefinisjon - Komplett Algoritme

En gyldig passering registreres når:

1. **Debounce-sjekk**: `tiden_siden_siste_passering >= debounce_ms` (server-tid)
2. **Sensorkorrelasjonsvindu**: Alle sensorbrudd innenfor `timeout_ms` grupperes (målestolpe-tid)
3. **Minimumskrav**: `antall_utløste_sensorer >= min_sensors`
4. **Enkelt duplikatfilter**: Flag-basert system forhindrer multiple K4-meldinger per passering
5. **K4-melding**: Sendes med server-tid som autoritativ tidsstempel (confirmation time)

### Duplikatfiltrering
Systemet implementerer enkel flag-basert duplikatforhindring for å sikre at samme fysiske passering kun sender én K4-melding:

```c
// Enkel flag-basert duplikatforhindring i passage_detection_t struct:
typedef struct {
    // ... eksisterende felter ...
    bool passage_sent_this_sequence;  // Flag for om K4 er allerede sendt
} passage_detection_t;

// Implementering i hovedlogikken:
if (passage_detectors[detector_idx].unique_sensors_count >= min_sensors_for_passage 
    && !passage_detectors[detector_idx].passage_sent_this_sequence) {
    
    // Send K4-melding kun første gang terskelen nås
    send_websocket_k4_message();
    passage_detectors[detector_idx].passage_sent_this_sequence = true;
}
```

**Hvordan det fungerer:**
- Når første sensor trigger → `passage_sent_this_sequence = false` (standard)
- Når `min_sensors` terskelen nås første gang → K4 sendes og flag settes til `true`
- Alle påfølgende sensorer i samme sekvens → **INGEN ny K4** siden flag allerede er `true`
- Ved timeout eller ny sekvens → flag resettes til `false` for neste passering

### Konfigurerbare Parametre (Aksesspunkt)

| Parameter | Type | Beskrivelse | Standardverdi | Område | NVS Lagret |
|-----------|------|-------------|---------------|--------|------------|
| `debounce_ms` | uint32_t | Minimum tid mellom separate passeringer | 1000ms | 500-15000ms | ✅ |
| `min_sensors` | uint8_t | Minimum antall sensorer som må utløses | 2 | 1-7 | ✅ |
| `timeout_ms` | uint32_t | Maks tid for sensorkorrelasjonsvindu | 500ms | 100-1000ms | ✅ |

### WebSocket Distribusjon (K=4 - Passering)
Ved registrert passering sendes følgende JSON til alle tilkoblede klienter:

```json
{
    "M": "aa:bb:cc:dd:ee:ff",  // MAC-adresse til målestolpe
    "K": 4,                    // Meldingstype: Passering
    "T": 1672531200,           // Server timestamp sekunder (confirmation time)
    "U": 123456,               // Server timestamp mikrosekunder (confirmation time)
    "S": 3                     // Antall sensorer som utløste
}
```

## Brukergrensesnitt (TimerView.vue)

### Timer-logikk
Brukergrensesnittet implementerer en state machine med følgende tilstander:

| Tilstand | Beskrivelse | Trigger |
|----------|-------------|---------|
| `ready` | Systemet er klart til å starte | Initial tilstand |
| `running` | Timer løper aktivt | Første passering mottatt |
| `finished` | Timer stoppet, resultat registrert | Andre passering mottatt |
| `error` | Feil i system eller tilkobling | Tap av tilkobling/sensorfeil |

### Passeringshåndtering
TimerView.vue reagerer på passeringer ved å overvåke `passages` array.

## Flytdiagram - Passeringsdeteksjon

```
Sensorbrudd (K=1, break_state=1)
              ↓
          MAC-matching
              ↓
         Debounce-sjekk
          ↙       ↘
       Innenfor    Utenfor debounce
      debounce         ↓
         ↓         Sequence-sjekk
      Ignorer        ↙       ↘
                 Eksisterende  Ny sekvens
                   sekvens       ↓
                     ↓      Opprett sequence
                Timeout?     (first_break_time)
                ↙     ↘           ↓
              Ja       Nei        ↓
               ↓        ↓         ↓
         Reset og   Legg til   Legg til
         ny sekvens  sensor    sensor
               ↓        ↓         ↓
               └────────┴─────────┘
                        ↓
                 Min_sensors oppnådd?
                    ↙       ↘
                 Nei         Ja
                  ↓           ↓
              Ignorer   Sjekk K4-flag
                         ↙       ↘
                    Allerede      Ikke
                     sendt       sendt
                       ↓           ↓
                   Ignorer   Send WebSocket (K=4)
                             + Sett flag = true
                                 ↓
                           TimerView.vue
                                 ↓
                        Start/Stopp timer
```

## Feilhåndtering og Robusthet

### Tilkoblingsstatus
Systemet overvåker kontinuerlig:
- **ESP-NOW peer status**: Målestolpe ↔ Aksesspunkt
- **WebSocket tilkobling**: Aksesspunkt ↔ Klient
- **Sensor-helse**: Deteksjon av blokkerte/defekte sensorer

### Graceful degradation
- Ved tap av enkelte sensorer: Systemet fortsetter med gjenværende sensorer
- Ved ESP-NOW problemer: Automatisk reconnect og peer re-registrering  
- Ved WebSocket disconnect: Automatisk reconnect med tilstandsgjenoppretting

### Data-integritet
- **Enkel duplikatforhindring**: Flag-basert system med mutex-beskyttelse
- **Tidsstempel-validering**: Sikrer kronologisk rekkefølge (server-tid)
- **Debounce-mekanisme**: Forhindrer falske triggere
- **Én K4 per passering**: Garanterer at samme passering kun sender én melding til frontend

## Systemkrav og Begrensninger

### Hardware-krav
- **ESP32 S3 WROOM 1U** mikrokontroller
- **Infrarød LED**: 9kHz PWM-modulert sender
- **Fotomottakere**: 7 stk analogt tilkoblede sensorer
- **LED-strip**: Status- og identifikasjonsvisning

### Ytelsesprofil
- **Maksimal detection rate**: 100Hz per sensor
- **Samtidige målestolper**: Opp til 8 enheter per aksesspunkt
- **WebSocket klienter**: Opp til 16 samtidige forbindelser
- **Passage throughput**: 200 passeringer/minutt maksimalt
- **Timing-presisjon**: ±1-2ms (ESP-NOW latency neglisjerbar)

### Nettverkskrav
- **WiFi 2.4GHz**: 802.11 b/g/n kompatibel
- **Båndbredde**: Minimum 1 Mbps for full funksjonalitet
- **Latens**: Maksimalt 50ms nettverk round-trip for optimal ytelse

## Utviklingsnotater og Kodestandarder

### Datastruktur-validering
Ved endringer av ESP-NOW datastruktur, sjekk alltid:
1. **Struct alignment**: Bruk `__attribute__((packed))` for nettverksmeldinger
2. **Byte-posisjonering**: Verifiser at parsing matcher struct-layout
3. **Navnekonvensjoner**: Bruk konsistente variabelnavn på tvers av moduler
4. **Størrelse-validering**: Kontroller at `sizeof()` matcher forventet størrelse

### Debugging og logging
For timing-problemer og ESP-NOW datastruktur:
```c
// Debug-logging av rå ESP-NOW data
ESP_LOGI(TAG, "📦 ESP-NOW pakke (len=%d):", len);
for (int i = 0; i < len; i++) {
    ESP_LOGI(TAG, "data[%d] = 0x%02X (%d)", i, data[i], data[i]);
}

// Presis logging av sensorendringer med dual timing
ESP_LOGI(TAG, "🔄 SENSORENDRING: sensor_id=%d, break_state=%d (%s)", 
         sensor_id, break_state, (break_state == 1) ? "BRUDD" : "GJENOPPRETTET");
ESP_LOGI(TAG, "   🕒 Målestolpe-tid: %u.%06u", sensor_time_sec, sensor_time_micros);
ESP_LOGI(TAG, "   🖥️ Server-tid: %u.%06u", server_time_sec, server_time_micros);

// Timeout-beregning logging
ESP_LOGI(TAG, "⏱️ Sequence timing: duration=%llu ms (limit=%d ms)", 
         sequence_duration_ms, sequence_timeout_ms);
```

### Testing og validering
- Test alle endringer med fysisk hardware
- Valider datastruktur med oscilloskop/debugger
- Kontroller at timing-krav opprettholdes
- Bekreft konsistent navngivning i hele kodebasen

---

*Dette dokumentet beskriver den funksjonelle designen av Timergate passeringsdeteksjon. For implementasjonsdetaljer og feilsøking, se kildekode og teknisk dokumentasjon.*
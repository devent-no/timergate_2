# Timergate Paring-prosess

Dette dokumentet beskriver den komplette paring-prosessen mellom TimerGate AP (Access Point) og målestolper i TimerGate-systemet.

## Oversikt

TimerGate bruker et **selvhelbredende discovery og paring-system** basert på ESP-NOW kommunikasjon. Systemet er designet for å være robust og håndtere alle edge-cases automatisk.

## Komponenter

- **TimerGate AP**: ESP32-basert access point som kjører webserver og WiFi hotspot
- **Målestolper**: ESP32-baserte sensorer som kommuniserer via ESP-NOW

## Discovery og Paring Prosess

### 1. Discovery-modus (Målestolpe søker system)

Når en målestolpe ikke er tilknyttet noe system:

```
Målestolpe tilstand:
- my_assigned_system_id = 0x00000000 (ikke tilknyttet)
- announcement_active = true
- Sender announce-meldinger hvert 5. sekund
- Sender IKKE sensordata (K=0)
```

**Announce-melding inneholder:**
- Device navn: "TimerGate Pole"
- MAC-adresse
- Firmware versjon
- RSSI estimat

### 2. AP Discovery

AP-en mottar announce-meldinger og:

```
1. Registrerer målestolpen i "discovered_poles" listen
2. Sender discovery-oppdatering til GUI
3. Målestolpen vises som "Tilgjengelig" i webgrensesnittet
```

### 3. Paring (System Assignment)

Når bruker trykker "Pair" i GUI:

```
1. AP sender system assignment via ESP-NOW til målestolpen
2. Assignment inneholder:
   - System ID (unikt for hver AP)
   - System navn (f.eks. "TimerGate System")
   - Target MAC (målestolpens adresse)

3. Målestolpe mottar assignment:
   - my_assigned_system_id = AP's system ID
   - announcement_active = false (stopper announce)
   - Lagrer assignment i NVS (permanent lagring)

4. AP flytter målestolpen fra "discovered" til "paired_poles"
5. Målestolpen starter sending av sensordata (K=0)
```

### 4. Normal drift

Når målestolpen er paret:

```
- Sender K=0 ADC-data hvert sekund
- Sender K=1 sensorbrudd-events ved passeringer
- Mottar konfigurasjonskommandoer fra AP
- Sender IKKE announce-meldinger
```

## Unpair-prosess

### 1. Manual Unpair (via GUI)

Når bruker trykker "Fjern" på en målestolpe:

```
1. AP sender CMD_UNPAIR kommando til målestolpen
2. AP fjerner målestolpen fra paired_poles (lokalt)
3. Lagrer endringen i NVS

Målestolpe ved mottak av CMD_UNPAIR:
- my_assigned_system_id = 0x00000000
- announcement_active = true
- Stopper sending av sensordata
- Starter announce-meldinger
- Lagrer endringen i NVS
- Dukker opp som "Tilgjengelig" igjen
```

### 2. Selvhelbredende Auto-korrigering

AP overvåker kontinuerlig for målestolper som sender data uten å være paret:

```
Hvis AP mottar sensordata fra ukjent målestolpe:
1. Logger: "AUTO-KORRIGERING: Målestolpe sender data men er ikke paired"
2. Sender automatisk CMD_UNPAIR til målestolpen
3. Ignorerer dataene til målestolpen går i discovery-modus

Dette løser situasjoner hvor:
- Målestolpen var paired, men AP glemte den (reset/config-endring)
- Network/ESP-NOW problemer under opprinnelig unpair
- Målestolpen har feil system assignment
```

## Teknisk Implementering

### ESP-NOW Kommando-struktur

```c
typedef struct {
    uint32_t system_id;      // AP's unike ID
    uint8_t msg_type;        // MSG_COMMAND (0x30)
    uint8_t target_mac[6];   // Målestolpens MAC
    uint8_t command_type;    // CMD_UNPAIR (0x0B)
    uint8_t data[];          // Kommando-spesifikk data
} command_msg_t;
```

### Spesiell CMD_UNPAIR Håndtering

CMD_UNPAIR kommandoer behandles **alltid**, uavhengig av system ID:

```c
// I målestolpens ESP-NOW receive callback:
if (command->command_type == CMD_UNPAIR) {
    // Aksepterer unpair uavhengig av system_id
    handle_esp_now_command(command, len);
    return;
}

// Andre kommandoer sjekkes for korrekt system_id
if (command->system_id != my_assigned_system_id) {
    return; // Ignorer
}
```

### K=0 Data Filtrering

Målestolpen sender kun sensordata når den er paret:

```c
// Send K=0 ADC-data kun hvis paret til system
if (!blink_mode && 
    current_status != STATUS_ERROR_SENSORS_BLOCKED && 
    esp_now_peer_added && 
    is_assigned_to_system()) {
    send_k0_adc_data_esp_now();
}
```

## Status-indikatorer

### Målestolpe LED-status

- **Initialisering**: Hvit LED-sekvens ved oppstart
- **Kalibrering**: Pulserende lilla under sensor-kalibrering  
- **Ready/Paret**: Grønn pulsering hver 60. sekund
- **Discovery**: Vanlig sensor-indikasjon (rødt ved brudd)
- **Identifikasjon**: Rask oransje blinking når "Identify" trykkes

### GUI-status

- **Tilgjengelige målestolper**: Vises på "Konfigurering"-siden
- **Parede målestolper**: Vises på "Enheter"-siden med signalkvalitet og status

## Robusthet og Feilhåndtering

### Selvhelbredende egenskaper

1. **ESP-NOW peer-registrering**: Automatisk registrering når målestolper sender data
2. **Auto-korrigering**: Automatisk unpair av målestolper som sender data uten å være paret
3. **Idempotent unpair**: Kan kjøres flere ganger uten skade
4. **NVS persistering**: Paring-status overlever restart på begge sider

### Edge-case håndtering

- **Målestolpe offline under unpair**: Auto-korrigering når den kommer online
- **AP reset**: Målestolper auto-korrigeres når de sender data
- **ESP-NOW rekkevidde**: Unpair fungerer så snart målestolpen er i rekkevidde
- **Dobbel-unpair**: Behandles elegant uten feil

## Feilsøking

### Målestolpe sender ikke announce

```bash
# Sjekk målestolpe-logger:
ESP_LOGI: "💾 System assignment status: ID=00000000, announce_active=true"

# Hvis ID != 00000000, send manual unpair eller restart målestolpen
```

### Målestolpe dukker ikke opp som tilgjengelig

```bash
# Sjekk AP-logger:
ESP_LOGI: "📢 Mottok pole announce via ESP-NOW"

# Hvis ikke mottatt, sjekk ESP-NOW peer-registrering og rekkevidde
```

### Auto-korrigering loop

```bash
# Sjekk at målestolpen stopper K=0 data etter unpair:
ESP_LOGI: "📊 Sender K=0 ADC-data" # Skal IKKE skje i discovery-modus

# Hvis det skjer, sjekk is_assigned_to_system() logikken
```

## Logging

### Viktige logg-meldinger

**AP-side:**
```
📢 Mottok announce fra målestolpe: [navn] (MAC: [mac])
🔧 AUTO-KORRIGERING: Målestolpe [mac] sender data men er ikke paired  
✅ System assignment sendt vellykket
🔓 Unpair fullført for målestolpe [mac]
```

**Målestolpe-side:**
```
📢 Sender pole announce (søker system-tilknytning)
✅ Mottok system assignment fra System ID: [id]
🔓 UNPAIR SPESIAL: Aksepterer unpair-kommando uavhengig av system ID
📢 Frigjort fra system - starter discovery-modus
```

## Sikkerhet

### System ID Isolering

- Hver AP har unikt System ID basert på MAC-adresse
- Målestolper aksepterer kun kommandoer fra sitt tilknyttede system
- **Unntak**: CMD_UNPAIR aksepteres alltid (for selvhelbredende egenskaper)

### MAC-adresse validering

- Alle kommandoer inneholder target MAC for å sikre korrekt adressering
- Broadcast-kommandoer brukes kun for discovery

## Ytelse

- **Announce-intervall**: 5 sekunder (lavt nettverkstrafikk)
- **Sensordata-intervall**: 1 sekund (rask respons)
- **Auto-korrigering**: Umiddelbar ved mottak av feil data
- **ESP-NOW latency**: < 10ms for kommandoer

---

*Dokumentasjon oppdatert: Januar 2025*  
*TimerGate versjon: 2.0*
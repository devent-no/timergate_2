# Timergate-systemet: Oversikt og installasjonsveiledning

Timergate er et tidtakersystem for sportsarrangementer, bestående av to målestolper og et aksesspunkt. Systemet bruker ESP32-S3 mikrokontrollere og kommunikasjon via ESP-NOW-protokollen.

## Overordnet systemarkitektur

Systemet består av følgende hovedkomponenter:

1. **Timergate Access Point (AP)**: 
   - Fungerer som sentralt kommunikasjonspunkt
   - Kjører en webserver som tilbyr brukergrensesnitt
   - Oppretter et WiFi-nettverk (SSID: "Timergate")
   - Håndterer kommunikasjon med målestolpene
   - Synkroniserer tid mellom komponenter
   - Samler data fra målestolpene

2. **Målestolper** (to stykker):
   - Utstyrt med sensorer (antagelig fotocellebaserte eller infrarøde)
   - Kommuniserer med AP via ESP-NOW
   - Rapporterer tidspunkt for "breaks" (når noen passerer mållinjen)
   - Mottar konfigurasjonsdata og synkroniseringskommandoer fra AP

3. **Brukergrensesnitt (GUI)**:
   - Vue 3-basert webapplikasjon
   - Bygges som statiske filer som lagres på AP's filsystem
   - Nås via nettleser på "timergate.local" eller IP-adresse
   - Viser data fra målestolpene i sanntid
   - Gjør det mulig å konfigurere systemet

## Dataflyt i systemet

1. **Oppsett og konfigurering**:
   - AP starter og oppretter WiFi-nettverk
   - Brukeren kobler til WiFi-nettverket med enhet (mobil/PC)
   - Bruker åpner webgrensesnittet i nettleseren
   - Konfigurering av målestolper skjer via GUI

2. **Tidtaking**:
   - Målestolpene registrerer når noen passerer
   - Data sendes til AP via ESP-NOW
   - AP mottar, behandler og lagrer data
   - Data vises i sanntid i webgrensesnittet
   - Websockets brukes for å oppdatere GUI i sanntid

3. **Tidsynkronisering**:
   - AP synkroniserer tiden med brukerens enhet via API
   - AP synkroniserer tid til målestolpene via ESP-NOW

## Teknisk oppbygning

### Timergate-AP (ESP32-basert AP og webserver)

- **Programvare**: ESP-IDF-basert firmware
- **Komponenter**:
  - HTTP-server for GUI og API
  - WebSocket-server for sanntidsoppdateringer
  - TCP-server for kommunikasjon med målestolpene
  - MDNS-tjeneste for "timergate.local" domene
  - ESP-NOW-grensesnitt for hurtig kommunikasjon med målestolpene
  - SPIFFS-filsystem for lagring av webfiler

### Timergate-GUI (Vue 3 webapplikasjon)

- **Teknologi**: Vue 3, JavaScript, HTML, CSS
- **Hovedkomponenter**:
  - `App.vue`: Hovedkomponent som håndterer brukergrensesnittet
  - `BreakItem.vue`: Komponent for visning av "breaks" (målpasseringer)
  - `Pole.vue`: Komponent for visning og konfigurering av målestolper
  - REST API-integrasjon for konfigurering
  - WebSocket-integrasjon for sanntidsoppdateringer

## Kommunikasjonsprotokoller

1. **ESP-NOW**: Brukes mellom AP og målestolpene for hurtig, lavnivå kommunikasjon
2. **HTTP REST API**: Brukes mellom webklienten og AP for konfigurasjon
3. **WebSockets**: Brukes for sanntidsoppdateringer fra AP til webklienten
4. **TCP Socket**: Brukes for direkte kommunikasjon mellom AP og målestolpene

## Blokkdiagram av systemet

```
┌─────────────────────────────────────────────────────┐
│                                                     │
│              Bruker med nettleser                  │
│                                                     │
└───────────────────┬─────────────────────────────────┘
                    │ HTTP/WebSocket
                    ▼
┌─────────────────────────────────────────────────────┐
│                                                     │
│               Timergate Access Point                │
│    ┌───────────────┐         ┌──────────────────┐   │
│    │  Web Server   │◄────────┤   SPIFFS         │   │
│    │  (GUI/API)    │         │   (GUI-filer)    │   │
│    └───────┬───────┘         └──────────────────┘   │
│            │                                        │
│    ┌───────┴───────┐         ┌──────────────────┐   │
│    │  WebSocket    │◄────────┤   TCP Server     │   │
│    │  Server       │         │                  │   │
│    └───────────────┘         └────────┬─────────┘   │
│                                       │             │
│    ┌───────────────┐                  │             │
│    │   ESP-NOW     │◄─────────────────┘             │
│    │  Controller   │                                │
│    └───────┬───────┘                                │
│            │                                        │
└────────────┼────────────────────────────────────────┘
             │ ESP-NOW
    ┌────────┴─────────┐
    │                  │
┌───▼──────────────┐ ┌─▼────────────────┐
│                  │ │                  │
│   Målestolpe 1   │ │   Målestolpe 2   │
│                  │ │                  │
└──────────────────┘ └──────────────────┘
```

## Bygging og installering av timergate-ap med GUI-filer

For å få systemet opp å kjøre på en ryddig måte, er det viktig å følge riktig prosedyre for bygging og flashting.

### Systemkrav
- macOS
- VS Code
- Git
- Node.js og npm
- ESP-IDF (Espressif IoT Development Framework)

### Portnavn for macOS
Port for AP: `/dev/cu.usbmodem2101`

### 1. Forberede GUI-filene
```bash
# Navigere til GUI-prosjektet
cd Software/timergate-gui

# Installere avhengigheter
npm install

# Bygge produksjonsversjonen av GUI
npm run build
```
Dette vil generere en `dist`-mappe med optimaliserte statiske filer.

### 2. Bygge og flashe AP-programvaren med GUI inkludert
```bash
# Navigere til AP-prosjektet
cd Software/timergate-ap

# Sette riktig målplattform (hvis ikke gjort tidligere)
idf.py set-target esp32s3

# Bygge hele prosjektet (inkludert GUI-filer fra dist-mappen)
idf.py build

# Flashe programvare til ESP32
idf.py flash -p /dev/cu.usbmodem2101
```

### 3. Overvåke systemet (valgfritt)
```bash
# Overvåke seriell output
idf.py monitor -p /dev/cu.usbmodem2101
```

### Viktige merknader:
- Systemet vil automatisk inkludere GUI-filene (fra dist-mappen) i flashingsprosessen
- Hvis `dist`-mappen ikke finnes, vil byggeprosessen feile med en tydelig feilmelding
- Partisjonsfilen `partitions_example.csv` definerer en dedikert "www" partisjon på 2MB for GUI-filene
- Aksesspunktet vil være tilgjengelig med SSID "Timergate" og passord "12345678"
- Webgrensesnittet vil være tilgjengelig på "http://timergate.local" eller via IP-adressen

## Anbefalinger for feilsøking
Hvis du støter på problemer:
1. Sjekk at `dist`-mappen faktisk er blitt opprettet etter bygging av GUI
2. Kontroller at `CONFIG_EXAMPLE_WEB_DEPLOY_SF=y` er satt i sdkconfig
3. Sjekk seriell output for eventuelle feilmeldinger
4. Kontroller at du har tilstrekkelig plass på ESP32-enheten (4MB flash anbefales)

Med disse stegene skal du være i stand til å bygge og flashe hele systemet uten problemer.

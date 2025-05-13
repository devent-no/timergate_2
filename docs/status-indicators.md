# Status Indikasjoner for Timergate

## Statusmønstre og betydning

| Status | Farge | Animasjonsmønster | Betydning |
|--------|-------|-------------------|-----------|
| `STATUS_INITIALIZING` | Hvit (64, 64, 64) | LED-er tennes én etter én, så slukkes | Systemet starter opp |
| `STATUS_WIFI_CONNECTING` | Blå (0, 0, 255) | Roterende LED (én aktiv LED som flytter seg) | Kobler til WiFi |
| `STATUS_SERVER_CONNECTING` | Gul (255, 255, 0) | Alle LED-er blinker samtidig (1 sek på/av) | Kobler til server |
| `STATUS_READY` | Grønn (0, 255, 0) | Kort "pulsering" hver 5. sekund | Systemet er klart, venter på aktivitet |
| `STATUS_ERROR_WIFI` | Rød (255, 0, 0) | Annenhver LED blinker skiftende | Feil med WiFi-tilkobling |
| `STATUS_ERROR_SERVER` | Rød (255, 0, 0) | Første og siste LED blinker | Feil med servertilkobling |
| `STATUS_ERROR_SENSORS_BLOCKED` | Rød (255, 0, 0) | Alle LED-er blinker (0,5 sek på/av) | Alle sensorer har vært blokkert i 5+ sekunder |
| `STATUS_CALIBRATING` | Lilla (128, 0, 128) | Pulserende LED for aktiv sensor, statiske LED-er for andre | Kalibreringsprosess (highpoint search) pågår |

## Statusoverganger

Systemet skifter automatisk mellom statusene i følgende tilfeller:

```
Oppstart → STATUS_INITIALIZING → STATUS_WIFI_CONNECTING
         → STATUS_SERVER_CONNECTING → STATUS_READY
```

Ved feil:
```
Tap av WiFi → STATUS_ERROR_WIFI
Tap av server → STATUS_ERROR_SERVER
Alle sensorer blokkert i 5+ sekunder → STATUS_ERROR_SENSORS_BLOCKED
```

Ved gjenoppretting fra feil:
```
Sensor-blokkering fjernet → STATUS_READY
WiFi gjenopprettet → STATUS_SERVER_CONNECTING
Server gjenopprettet → STATUS_READY
```

Ved kalibrering:
```
Manuell kalibrering startet → STATUS_CALIBRATING
Automatisk kalibrering etter oppstart → STATUS_CALIBRATING
Kalibrering fullført → STATUS_READY
```

## Operasjon i normaldrift

I `STATUS_READY` (normal drift) vil systemet:
1. Vise et kort grønt blink med alle LED-er hvert 5. sekund
2. Ellers vise normal sensoraktivitet i hvitt lys når sensorer utløses

## Highpoint Search-prosess (Kalibrering)

Under `STATUS_CALIBRATING` viser systemet kalibreringsprosessen med tydelige visuelle indikasjoner:

1. **Aktiv sensor under kalibrering**: Pulserende lilla lys som varierer i intensitet fra 50 til 250 i en 1-sekunds syklus
2. **Tidligere kalibrerte sensorer**: Fast grønn farge (0, 100, 0)
3. **Sensorer som venter på kalibrering**: Dempet lilla farge (20, 0, 20)

Kalibreringsprosessen går gjennom hver sensor i rekkefølge (0-6) og søker etter optimal offset-verdi for hver sensor. Når en sensor nærmer seg fullført kalibrering (mer enn 95% ferdig), veksler LED-en mellom hvitt og grønt lys for å indikere at kalibreringen for den sensoren snart er fullført.

Når alle sensorer er kalibrert, vil systemet:
1. Lagre de nye innstillingene i NVS (non-volatile storage)
2. Publisere de nye innstillingene til serveren
3. Gå tilbake til `STATUS_READY`-tilstand med grønn pulsering

Hvis kalibreringen ikke klarer å finne gyldige verdier for noen sensorer, settes standardverdier (break_limit = 1000, offset = 4500) for disse sensorene.

## LED-fargekoder

Fargene er valgt for å være intuitive og synlige på avstand:

- **Hvit**: Normal drift / informasjon
- **Blå**: Pågående prosess - WiFi-oppsett
- **Gul**: Venter på servertilkobling
- **Grønn**: Klar / vellykket tilkobling
- **Rød**: Feil / problem som krever oppmerksomhet
- **Lilla**: Kalibreringsprosess pågår

## Kalibreringsprosess i detalj

Kalibreringsprosessen (highpoint search) følger disse trinnene:

1. **Initialisering**: Systemstatus settes til `STATUS_CALIBRATING` med `highpoint_channel = 0`
2. **Per sensor**:
   - Systemet justerer offset-verdien for sensoren i trinn på 300
   - For hver offset-verdi, måles ADC-verdien
   - Maksimal ADC-verdi og tilhørende offset lagres
3. **Avslutning av sensorkalibrering**:
   - Når hele offset-området er utforsket (0-8092), settes den optimale offset-verdien
   - Break-grensen settes til 70% av maksimal ADC-verdi
   - Systemet går videre til neste sensor
4. **Fullføring**:
   - Når alle sensorer er kalibrert, lagres innstillingene i NVS
   - Innstillingene sendes til serveren
   - Systemstatus settes tilbake til `STATUS_READY`

## Fordeler for operatøren

Dette statusindikasjonsystemet gir operatøren flere fordeler:
- Tydelig visuell bekreftelse på at systemet fungerer (grønne blink)
- Umiddelbar indikasjon på hvilken type problem som har oppstått (ulike røde mønstre)
- Synlig av at systemet går gjennom oppstartsekvensen (distinkte mønstre)
- Mulighet til å verifisere korrekt operasjon fra avstand
- Tydelig indikasjon på kalibreringsfremdrift (lilla lys med varierende intensitet)

## Teknisk implementasjon

Statusindikasjonsystemet er implementert gjennom:
- En status-enum med alle definerte tilstander
- En dedikert funksjon `handle_status_animation()` for å håndtere alle animasjoner
- Tilstandsoverganger i relevante deler av koden (WiFi-tilkobling, servertilkobling, sensorkontroll)
- LED-kontroll med fullt RGB-fargespekter
- Funksjon `set_system_status(status)` for å endre systemstatus og initialisere animasjoner

## Konfigurasjon

Parameterne for statusindikasjoner kan justeres i koden:
- Blinkefrekvenser (`step_time`, `blink_cycle`, `pulse_cycle`)
- Fargeverdier i `STATUS_COLORS` tabellen
- Tidsterskler (f.eks. `min_time_for_alert` for sensorblokkeringsvarsling)
- Pulseringsparametere for kalibrering (`pulse_period`, `brightness`)

## Automatisk kalibrering

Systemet har støtte for automatisk kalibrering:
1. Etter oppstart og tilkobling går systemet til `STATUS_READY`
2. Etter 5 sekunder i `STATUS_READY` starter automatisk kalibrering
3. Systemstatus endres til `STATUS_CALIBRATING` og kalibreringsprosessen starter
4. `auto_calibration_started`-flagget settes for å unngå gjentatt kalibrering

Operatøren kan også manuelt starte kalibrering ved å sende en "hsearch"-kommando til systemet. Dette vil umiddelbart sette systemet i `STATUS_CALIBRATING`-tilstand og starte kalibreringsprosessen.
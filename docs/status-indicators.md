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

## Operasjon i normaldrift

I `STATUS_READY` (normal drift) vil systemet:
1. Vise et kort grønt blink med alle LED-er hvert 5. sekund
2. Ellers vise normal sensoraktivitet i hvitt lys når sensorer utløses

## LED-fargekoder

Fargene er valgt for å være intuitive og synlige på avstand:

- **Hvit**: Normal drift / informasjon
- **Blå**: Pågående prosess - WiFi-oppsett
- **Gul**: Venter på servertilkobling
- **Grønn**: Klar / vellykket tilkobling
- **Rød**: Feil / problem som krever oppmerksomhet

## Fordeler for operatøren

Dette statusindikasjonsystemet gir operatøren flere fordeler:
- Tydelig visuell bekreftelse på at systemet fungerer (grønne blink)
- Umiddelbar indikasjon på hvilken type problem som har oppstått (ulike røde mønstre)
- Synlig av at systemet går gjennom oppstartsekvensen (distinkte mønstre)
- Mulighet til å verifisere korrekt operasjon fra avstand

## Teknisk implementasjon

Statusindikasjonsystemet er implementert gjennom:
- En status-enum med alle definerte tilstander
- En dedikert funksjon `handle_status_animation()` for å håndtere alle animasjoner
- Tilstandsoverganger i relevante deler av koden (WiFi-tilkobling, servertilkobling, sensorkontroll)
- LED-kontroll med fullt RGB-fargespekter

## Konfigurasjon

Parameterne for statusindikasjoner kan justeres i koden:
- Blinkefrekvenser (`step_time`, `blink_cycle`, `pulse_cycle`)
- Fargeverdier i `STATUS_COLORS` tabellen
- Tidsterskler (f.eks. `min_time_for_alert` for sensorblokkeringsvarsling)
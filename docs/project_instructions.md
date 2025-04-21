# Timergate-prosjekt: Instruksjoner for samarbeid

## Prosjektinformasjon
- **GitHub-repo**: https://github.com/larslys/timergate-test
- **Systemdokumentasjon**: Se SYSTEM_OVERVIEW.md for detaljer om systemarkitektur, dataflyt og installasjonsveiledning
- **Beslutningslogg**: Se DECISIONS.md for viktige beslutninger tatt i prosjektet

## Hovedkomponenter
- **timergate-ap**: ESP32-programvare som kjører WiFi access point og webserver
- **timergate-gui**: Vue 3-basert brukergrensesnitt som serveres fra ESP32

## Hardware
- ESP32 S3 WROOM 1U mikrokontroller
- To målestolper med sensorer
- Ett aksesspunkt

## Vedlikehold av prosjektdokumentasjon
- **SYSTEM_OVERVIEW.md** skal oppdateres ved:
  - Endringer i systemarkitektur
  - Nye komponenter eller kommunikasjonsprotokoller
  - Endringer i byggeprosessen
  - Oppdateringer i installasjonsanvisninger

- **DECISIONS.md** skal oppdateres når vi tar beslutninger om:
  - Arkitekturvalg
  - Teknologivalg
  - Funksjonalitetsendringer
  - Løsninger på tekniske problemer
  - Prioriteringer

## Oppstart av nye samtaler
Ved oppstart av nye samtaler, referer til disse instruksjonene for å raskt komme i gang med produktivt arbeid.


## Diagnostikk og feilsøking

Timergate-systemet har innebygde verktøy for diagnostikk og feilsøking:

- **/debug** endepunkt: Viser informasjon om filsystemet og tilkoblede målestolper
- **/ws_test** endepunkt: Gir et enkelt testgrensesnitt for WebSocket-kommunikasjon
- **/api/v1/debug/simulate** endepunkt: Genererer simulerte data fra en virtuell målestolpe, nyttig for testing uten fysisk hardware

Ved feilsøking av MAC-adressehåndtering, sjekk følgende:
1. Hvilke nominelle MAC-adresser er registrert i systemet (/debug)
2. Om kommandoer sendes til riktig målestolpe (via monitor-outputen)
3. Om WebSocket-data inneholder forventede MAC-adresser (via /ws_test)
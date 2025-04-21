# Timergate GUI Design Guide

## Overordnet struktur
Timergate GUI er bygget med en modulær tilnærming med følgende hovedkomponenter:

- **Dashboard**: Oversiktsside med nøkkelinformasjon og hurtigtilgang til sentrale funksjoner
- **Timer**: Detaljert kontroll av tidtaking, sensorer og kalibrering
- **Enheter**: Skanning, tilkobling og administrasjon av målestolper
- **Konfigurering**: Systeminnstillinger og nettverkskonfigurasjon
- **Logg**: Historikk over tidtakingshendelser

Grensesnittet har to moduser:
1. **Utviklingsvisning**: Enkel visning for testing og utvikling
2. **Produksjonsvisning**: Fullstendig, brukerorientert grensesnitt for sluttbrukere

## Komponentbeskrivelser

### MainLayout
Fungerer som container for alle visninger i produksjonsgrensesnittet. Inkluderer:
- Toppmeny med navigasjon mellom hovedvisninger
- Konsistent styling på tvers av alle visninger
- Innebygd veksling mellom utviklings- og produksjonsvisning

### Dashboard
Viser systemets status på høyt nivå:
- Gjeldende tid
- Antall tilkoblede målestolper
- Aktive sensorer
- Siste registrerte tidtakingshendelser
- Hurtigknapper for vanlige handlinger

### Timer-kontroll
Detaljert styring av tidtakingen:
- Stor tidteller
- Start/stopp-kontroller
- Manuell justering av straffer og feil
- Visuell indikasjon på hvilke sensorer som er aktive

### Enheter
Administrasjon av tilkoblede enheter:
- Skanning etter nye enheter
- Tilkobling/frakobling av enheter
- Status for hver målestolpe (batteri, signalstyrke, etc.)
- Kalibreringskontroller per enhet

### Konfigurering
Systeminnstillinger:
- Wi-Fi-konfigurasjon (egen nettverksmodus eller tilkobling til eksisterende nettverk)
- Brukeradministrasjon
- Tidtakingsinnstillinger (forsinkelser, tidsvinduer, etc.)
- Firmware-oppdatering

### Logg
Historikk over hendelser:
- Filtrering og sortering av hendelser
- Eksport av data
- Statistikk og oppsummering

## Farger og styling
- **Primærfarge**: #0078D7 (blå)
- **Sekundærfarge**: #f5f5f5 (lys grå)
- **Aksent**: #28a745 (grønn for positive hendelser), #dc3545 (rød for advarsler/feil)
- **Typografi**: Sans-serif fonter (Arial, Segoe UI)
- **Komponenter**: Runde hjørner, subtile skygger for dybde

## Fremtidige utviklingsplaner
1. Fullføre grunnleggende komponentstruktur
2. Implementere responsivt design for mobil/tablet bruk
3. Legge til enhets-spesifikk kalibrering
4. Utvide logging med eksport-funksjoner
5. Implementere direkte integrasjon med konkurransesystemer
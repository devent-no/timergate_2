# Passeringsdeteksjon i Timergate

## Konsept

I Timergate-systemet definerer vi to viktige begreper:

1. **Brudd**: Et enkelt sensorbrudd på en målestolpe
2. **Passering**: En filtrert hendelse som indikerer at en hund har passert målstolpen

En passering registreres når et konfigurerbart minimumsantall unike sensorer på samme målestolpe utløses innenfor den konfigurerte sekvens-timeout-perioden (standard: 1000 ms).

## Funksjonalitet

### Passeringsdeteksjon
- Systemet teller antall unike sensorer som utløses på hver målestolpe
- Når antallet når eller overstiger minimumsgrensen, registreres en passering
- Tidspunktet for passeringen settes til tidspunktet for det første sensorbruddet i sekvensen
- Systemet sender informasjon om passeringen til GUI for visning og registrering

### Debouncing
- Etter at en passering er registrert, ignorerer systemet alle nye brudd på samme målestolpe i debounce-perioden
- Dette forhindrer at vibrasjoner eller andre forstyrrelser registreres som separate passeringer
- Når debounce-perioden er over, er systemet klart til å registrere en ny passering

### Tidssynkronisering og tidshopp
- Systemet bruker tidsstempler fra sensorer for å beregne hvor lang tid det har gått mellom sensorbrudd
- Ved tidshopp (når tidsstempler plutselig hopper bakover) fortsetter systemet sekvensen
- I stedet for å starte en ny sekvens ved tidshopp, antas en kort tid mellom sensorer (standard: 200ms)
- Dette gir mer robust deteksjon når sensorer utløses i rask rekkefølge

### Konfigurerbare parametere
- **debounce_tid**: Tidsperioden (i millisekunder) mellom passeringer på samme målestolpe
- **min_sensor_count**: Minimum antall unike sensorer som må utløses for å registrere en passering
- **sequence_timeout**: Maksimal tid (i millisekunder) mellom første og siste sensorbrudd i en gyldig passeringssekvens

## Implementasjon

### Datastrukturer
```c
typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];  // MAC-adresse til målestolpen
    uint32_t first_break_time;      // Tidspunkt for første brudd i sekvensen (sekunder)
    uint32_t first_break_micros;    // Mikrosekunder del
    uint32_t last_passage_time;     // Tidspunkt for siste registrerte passering
    uint32_t last_passage_micros;   // Mikrosekunder del
    int sensor_count;               // Antall unike sensorer med brudd i gjeldende sekvens
    bool sensors_triggered[MAX_SENSORS_PER_POLE]; // Hvilke sensorer som er utløst
    uint32_t last_sensor_time;      // Siste tidspunkt fra sensoren
    uint32_t last_sensor_micros;    // Mikrosekunder del
    uint32_t sequence_start_time;   // Starttidspunkt for gjeldende sekvens
} passage_detection_t;
```

### Logikk for passeringsdeteksjon
1. Når et sensorbrudd mottas:
   - Sjekk om debounce-perioden er aktiv for målestolpen
   - Hvis ja, ignorer bruddet
   - Hvis nei, fortsett prosesseringen

2. Beregn tid siden forrige sensor:
   - Hvis tiden går fremover, beregn faktisk tid
   - Hvis tiden hopper bakover, bruk en fast antatt tid (200ms) og fortsett sekvensen

3. Hvis sensoren ikke allerede er registrert for denne målestolpen:
   - Øk sensortelleren
   - Marker sensoren som utløst
   - Hvis dette er det første bruddet i sekvensen, lagre tidspunktet

4. Sjekk om sekvensen har tidsavbrutt:
   - Hvis tiden siden sekvensstart overstiger sequence_timeout, tilbakestill sekvensen

5. Hvis sensortelleren når minimumsgrensen:
   - Registrer en passering med tidspunktet for første brudd
   - Send passeringsinformasjon til GUI
   - Start debounce-perioden for målestolpen

### Tilbakestilling
- Når debounce-perioden utløper, er systemet klart for en ny passering
- Når sekvens-timeouten utløper, tilbakestilles sensortelleren og listen over utløste sensorer for målestolpen

## Anbefalte innstillinger

Med vertikalt plasserte sensorer og raskt bevegende hunder:
- **debounce_tid**: 1000ms (minimumstid mellom separate passeringer)
- **min_sensor_count**: 2-4, avhengig av ønsket nøyaktighet
- **sequence_timeout**: 500-1000ms (maksimal tid for en enkel passering)

Disse innstillingene kan justeres basert på faktiske forhold som:
- Hastigheten på hundene
- Avstanden mellom sensorene
- Ønsket sensitivitet i registreringen

## Fordeler med denne løsningen
- Robust mot falske utløsninger (krever flere sensorer)
- Nøyaktig tidtaking (bruker tidspunktet for første brudd)
- Fleksibel (konfigurerbare parametere)
- Tolerant for tidshoppavvik i sensorkommunikasjonen
- Ingen duplikate registreringer (debounce-periode)
- Enkel å forstå og vedlikeholde

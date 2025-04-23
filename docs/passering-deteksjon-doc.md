# Passeringsdeteksjon i Timergate

## Konsept

I Timergate-systemet definerer vi to viktige begreper:

1. **Brudd**: Et enkelt sensorbrudd på en målestolpe
2. **Passering**: En filtrert hendelse som indikerer at en hund har passert målestolpen

En passering registreres når et konfigurerbart minimumsantall (standard: 2) unike sensorer på samme målestolpe utløses innenfor den konfigurerte debounce-perioden (standard: 1000 ms).

## Funksjonalitet

### Passeringsdeteksjon
- Systemet teller antall unike sensorer som utløses på hver målestolpe
- Når antallet når eller overstiger minimumsgrensen, registreres en passering
- Tidspunktet for passeringen settes til tidspunktet for det første sensorbruddet i sekvensen
- Systemet sender informasjon om passeringen til GUI for visning og registrering

### Debouncing
- Etter at en passering er registrert, ignorerer systemet alle nye brudd på samme målestolpe i debounce-perioden
- Dette forhindrer at vibrasjoner eller andre forstyrrelser registreres som separate passeringer
- Når debounce-perioden er over, nullstilles sensortelleren for målestolpen

### Konfigurerbare parametere
- **debounce_tid**: Tidsperioden (i millisekunder) mellom passeringer på samme målestolpe
- **min_sensor_count**: Minimum antall unike sensorer som må utløses for å registrere en passering

## Implementasjon

### Datastrukturer
```c
typedef struct {
    uint8_t mac[ESP_NOW_ETH_ALEN];  // MAC-adresse til målestolpen
    uint32_t first_break_time;      // Tidspunkt for første brudd i sekvensen (sekunder)
    uint32_t first_break_micros;    // Mikrosekunder del
    uint32_t last_passing_time;     // Tidspunkt for siste registrerte passering
    uint32_t last_passing_micros;   // Mikrosekunder del
    int sensor_count;               // Antall unike sensorer med brudd i gjeldende sekvens
    bool sensors_triggered[MAX_SENSORS_PER_POLE]; // Hvilke sensorer som er utløst
} pole_passing_t;
```

### Logikk for passeringsdeteksjon
1. Når et sensorbrudd mottas:
   - Sjekk om debounce-perioden er aktiv for målestolpen
   - Hvis ja, ignorer bruddet
   - Hvis nei, fortsett prosesseringen

2. Hvis sensoren ikke allerede er registrert for denne målestolpen:
   - Øk sensortelleren
   - Marker sensoren som utløst
   - Hvis dette er det første bruddet i sekvensen, lagre tidspunktet

3. Hvis sensortelleren når minimumsgrensen:
   - Registrer en passering med tidspunktet for første brudd
   - Send passeringsinformasjon til GUI
   - Start debounce-perioden for målestolpen

### Tilbakestilling
- Når debounce-perioden utløper, tilbakestill sensortelleren og listen over utløste sensorer for målestolpen
- Dette gjør systemet klart til å registrere en ny passering

## Fordeler med denne løsningen
- Robust mot falske utløsninger (krever flere sensorer)
- Nøyaktig tidtaking (bruker tidspunktet for første brudd)
- Fleksibel (konfigurerbare parametere)
- Ingen duplikate registreringer (debounce-periode)
- Enkel å forstå og vedlikeholde

# CANSAT – TÖMÖRÍTETT TELEMETRIA + LORA RENDSZER


**Fontos:** az oldal alján részletes *Fogalomtár* található. A dokumentum megértéséhez elengedhetetlen végigolvasni, ezért mindenki nyugodjon le a picsába, és ha valami nem világos, először ott nézze meg a jelentését. :)

## 1. Mintavételezés
A CanSat 2 Hz frekvenciával mér (0,5 másodpercenként).  
Minden mérési eredményből egy tömörített mintapack készül (5–8 byte).

A mintavételezés a teljes küldetés alatt azonos marad, a CanSat fedélzeti kódjában fixen 2 Hz-re állítva. A 2 Hz-es mintavétel minden szenzorra érvényes, de nem minden szenzorértéket küldünk le minden mintánál, csak amit a protokoll fontosnak ítél.

A szenzorok, amelyek adatai bekerülhetnek a telemetriába:
- Nyomás / magasság (BME280/BMP585)
- Hőmérséklet, relatív páratartalom (BME/BMP + kiegészítő szenzorok)
- Levegőminőség – PM2.5 (SPS30)
- CO₂ (SCD40)
- UV / fény / IR (SI1145)
- Irány, dőlés, forgás (BNO085 – yaw/pitch/roll)
- GPS pozíció, magasság, irány (BE-880Q)


A telemetria protokoll úgy van kialakítva, hogy a mostani alap (T/RH/P) mellett fokozatosan bővíthető legyen ezekkel az adatokkal, anélkül hogy a LoRa link stabilitása romlana.


## 1.1. Fedélzeti MCU feladatai – nagyfrekvenciás mintavételezés és előfeldolgozás

A CanSat fedélzeti mikrokontrollere nem csak a 2 Hz-es telemetriai mintákat készíti elő, hanem **nagyobb frekvencián (akár 10 Hz-en)** folyamatosan méri a szenzorokat, és az adatokból **előfeldolgozott, zajszűrt** értékeket állít elő.

A fedélzeti MCU főbb feladatai:

- **10 Hz-es nyers mintavételezés** minden releváns szenzorról (T/RH/P, PM2.5, UV, CO₂ stb.)
- **mozgóátlagolás (moving average)** 5–10 minta alapján  
  – kiszűri a rövid, szenzorhibából vagy turbulenciából eredő tüskéket  
- **szélsőértékek eldobása** (min/max clipping)  
  – megakadályozza, hogy egy hibás minta befolyásolja a telemetriát
- **2 Hz-es „telemetriai minták” előállítása** a 10 Hz-es nyers adatokból
- **változásdetektálás előkészítése**: a tömörített pack-ekhez jelzi, hogy mely paraméterek változtak
- **batch gyűjtés** 8–12 mintáig (attól függően, hogy mikor érjük el a 48 byte-os payload méretet)
- **LoRa csomag összeállítása** base_MET + bitmask + pack-ek

Ennek eredményeként a telemetria:

- **sokkal stabilabb**, mint a nyers adatok,
- **több tájékoztató értékkel** rendelkezik (nincs szenzorzaj),
- **kevesebb bitet** igényel a változásdetektálás miatt,
- és a földi állomás grafikonjai **tiszta, sima légköri profilt** mutatnak.


A 10 Hz → 2 Hz előfeldolgozás tehát nem vesz el információt – éppen ellenkezőleg:  
**minden fontos trendet megtart, miközben eltünteti a zajt és csökkenti az adatforgalmat.**


### Fedélzeti MCU – adatfolyam szemléltető ábra

```
   ┌──────────────┐        10 Hz nyers minták         ┌─────────────────────┐
   │  Szenzorok   │ ─────────────────────────────────▶│  Nyers adat buffer  │
   │ T / RH / P   │                                   └─────────────────────┘
   │ PM2.5 / UV   │
   │ CO₂ / BNO085 │
   └──────────────┘                                             ▼   

               ┌─────────────────────────────── Mozgóátlag (MA 5–10 minta) ────────────────────────────────┐
               ▼                                                                                           │
       ┌────────────────────┐                                                                              │
       │ Szélsőérték-szűrés │◀───────────────────── hibás tüskék eldobása ─────────────────────────────────┘
       └────────────────────┘
               ▼
       ┌────────────────────────────┐
       │ 2 Hz telemetriai minták    │  (stabilizált, zajmentes érték)
       └────────────────────────────┘
               ▼
       ┌────────────────────────────┐
       │  Változásdetektálás (flag) │
       │       bitmask előkészítés  │
       └────────────────────────────┘
               ▼
       ┌────────────────────────────┐
       │   Pack-ek gyártása (6–8 B) │
       └────────────────────────────┘
               ▼
       ┌────────────────────────────┐
       │ Batch gyűjtés (8–12 pack)  │
       │ + base_MET meghatározása   │
       └────────────────────────────┘
               ▼
       ┌────────────────────────────┐
       │   LoRa SF12 csomag (≤48 B) │
       └────────────────────────────┘
```

## 2. Időbélyegzés – MET
- Csomagonként több minta megy le.
- Az első minta ideje: base_MET (2 byte).
- 1 tick = 0,5 s.

A MET egy 16 bites számláló, amely a repülés elején 0-ról indul, és minden 0,5 másodperces lépésnél 1-gyel növekszik. Túlfutás esetén (65535 után) visszaugrik 0-ra, de a CanSat tipikus repülési ideje (néhány perc) messze a túlfutás előtt véget ér, így ez a gyakorlatban nem okoz problémát.

A földi állomás minden beérkezett csomagnál elmenti a base_MET-et, és ehhez képest számolja ki az adott csomagban lévő minták időpontjait (index alapján: base_MET + minta_sorszám).

### Példa – base_MET szerepe és a minták időrekonstrukciója

A telemetria minden csomagja tartalmaz egy **base_MET** mezőt, amely 2 byte-on a csomag első mintájának időbélyegét (0,5 s egységekben) tárolja. A csomagban szereplő többi minta időbélyege a mintasorszámtól függ:

```
Időbélyeg = base_MET + index
```

A következő táblázat egy példát mutat 6 mintára:

```
Idx | base_MET | MET (valódi idő) | Temp | RH | P   | PM2.5 | UV
-----------------------------------------------------------------
0   | 120      | 60,0 s           | 15.2 | 81 | 948 | 13.5  | 3
1   | 120      | 60,5 s           | 15.2 | 81 | 948 | 13.5  | 3
2   | 120      | 61,0 s           | 15.3 | 81 | 948 | 13.5  | 3
3   | 120      | 61,5 s           | 15.3 | 80 | 948 | 13.0  | 2
4   | 120      | 62,0 s           | 15.3 | 80 | 948 | 13.0  | 2
5   | 120      | 62,5 s           | 15.4 | 80 | 947 | 12.5  | 2
```

A földi állomás ezekből az adatokból a teljes idősort hibamentesen visszaállítja, akkor is, ha a rádión érkezett csomagok esetleg késnek vagy kimaradnak.



### Miért nem 0 a base_MET?

A base_MET **nem a küldetés indulásának idejét jelzi**, hanem azt, hogy **a csomagban található első mintát mikor vettük**, 0,5 másodperces tickekben mérve. Ezért a base_MET értéke szinte soha nem lesz 0, csak a legelső csomagnál.

**Mikor indul a MET számláló?**

A MET a küldetés elején **0-ról indul**, ez jellemzően:
- vagy a rakéta leválásának pillanata,
- vagy a CanSat saját bekapcsolt állapotban lévő időzítőjének „T0” pontja.

A számláló ezután minden **0,5 másodperc** elteltével eggyel nő:

```
000 → 0,0 s  
001 → 0,5 s  
002 → 1,0 s  
003 → 1,5 s  
...  
```

Ez folytatódik egészen addig, amíg a CanSat mér — a földi állomás pedig a MET alapján pontos, fél másodperces időskálán tudja visszarajzolni a légköri adatokat.

**Miért 120 a fenti példában?**  
- 1 tick = 0,5 s  
- 120 tick = 120 × 0,5 = **60,0 s**

Vagyis a csomag első mintája a repülés 60. másodpercében készült.  
Ez teljesen természetes, mert a CanSat már régen mér, és a rádió csak ekkor küldte le a batch-et.

**A minták időpontja így számolódik:**
```
MET = base_MET + index
Idő másodpercben = MET × 0,5
```

**Kiegészítő példa – ha a csomag később érkezik:**

Ha base_MET = 245:

- MET = 245  
- Idő = 245 × 0,5 = **122,5 s**

```
Idx | base_MET | MET (valódi idő)
---------------------------------
0   | 245      | 122,5 s
1   | 245      | 123,0 s
2   | 245      | 123,5 s
3   | 245      | 124,0 s
```

A földi állomás így akkor is tökéletes idősort állít vissza, ha a csomag **késve**, vagy **csúszva** érkezik.  
Ez a telemetria rendszer egyik legfontosabb alapköve.


## 3. Tömörített mintastruktúra (T / RH / P)

### Hőmérséklet (Temp)
- Felbontás: 0,1 °C  
- Tárolás: 7 bit egész + 4 bit tizedes  
- Tartomány: –40,0 … +87,9 °C  

A 0,1 °C-os felbontás azért elegendő, mert a légkörben a hőmérséklet változása kb. 0,026 °C körül van 4 méteres magasságkülönbségre. Ennél finomabb felbontás a szenzorzaj és a mérési bizonytalanság miatt már nem adna több hasznos információt, viszont több bitet foglalna.

### Relatív páratartalom (RH)
- Felbontás: 1%  
- Tárolás: 7 bit  
- Tartomány: 0–100%  

A páratartalmat egész százalékban kezeljük, mert a tizedszázalékos változások nagy része inkább zaj, mint valódi fizikai változás, így felesleges lenne tizedes pontossággal továbbítani.

### Nyomás (P) – 822…1077 hPa
- 1 byte: 0…255 → 822…1077 hPa  
- Tizedes: 4 bit (0,1 hPa)

A 822…1077 hPa tartomány lefedi a várható repülési magasságokat: kb. 1 km környékén 900–900+ hPa, talajközelben 980–1030 hPa körüli értékekre számítunk. A 0…255 közötti kódolt érték a 822 hPa-hoz képest eltolva tárolódik, így a 255 lépcső kényelmesen lefedi ezt a 256 hPa széles tartományt.

### PM2.5
- 14 bit  
- 0,5 µg/m³ felbontás  

### CO₂
- 1–2 byte  
- egész vagy 0,1 ppm  

### Irány (BNO085)
- 2 byte  
- egész fok  

### UV index
- 1 byte  

### Összefoglaló táblázat – telemetriai mezők

Az egyes főbb paraméterekhez tartozó bitméret és indoklás:

| Paraméter               | Mennyit küldünk?      | Miért?                                  |
|-------------------------|-----------------------|-----------------------------------------|
| Nyomás (hPa)            | 12 bit                | kell a finom magassági profilhoz        |
| Hőmérséklet (°C)        | 11 bit                | 0,1 °C felbontás elég, nem kell több    |
| Páratartalom (%)        | 7 bit                 | egész érték, tizedes nem szükséges      |
| PM2.5                   | 14 bit                | 0,5 µg/m³ felbontás bőven elég          |
| CO₂ (ppm)               | 8–16 bit              | a küldetés igényei szerint választható  |
| Irány (yaw/pitch/roll)  | 16 bit / csatorna     | egész fok pontosság elegendő            |
| UV index                | 8 bit                 | egész érték, 0–255 között bőven elég    |   


### Teljes mintapack
5 byte (T/RH/P) + opcionális mezők → 6–8 byte.

## 4. Bitmask – változásdetektálás
- 1 = változott → a pack-et elküldjük  
- 0 = nem változott → ismétlés a földi oldalon  
- Bitmask mérete: 1–2 byte (N mintaszámtól függ)

A bitmask bitjei az adott csomagban szereplő mintákhoz tartoznak, például az LSB (legalsó bit) a 0. indexű mintához, a következő bit az 1. indexűhöz stb. Így egyetlen byte akár 8 minta állapotát is leírja. Ha a projekt során úgy döntünk, hogy ritkábban küldünk nagyobb csomagokat (például 10–12 mintával), akkor a bitmask mérete automatikusan 2 byte-ra nő.   

### Hogyan számolja ki a vevő a bitmask hosszát?   

A vevő oldalon a bitmask méretét az N határozza meg. És mi N-t explicit elküldjük minden csomagban.   


## 5. LoRa csomag felépítése
```
[ base_MET (2B) ]
[ N (1B) ]
[ bitmask (1–2B) ]
[ pack-ek (csak flag=1 esetén) ]
```

### Részletes csomagpélda (hexadecimális bontással)

Az alábbi példa bemutatja, hogyan néz ki egy valódi rádiócsomag SF12-ben, ha 6 mintából 4 változott.

```
base_MET = 0x0078        (dec: 120 → 60,0 s)
N        = 0x06          (6 minta)
bitmask  = 0b00101101    (LSB a 0. minta)
```

A ténylegesen továbbított pack-ok (lásd az adattáblázat sorait fent, a példában):

```
pack0 → T/RH/P/PM25/UV
pack2 → T/RH/P/PM25/UV
pack3 → T/RH/P/PM25/UV
pack5 → T/RH/P/PM25/UV
```

A rádiópayload így épül fel:

```
[78 00]     base_MET (little-endian)
[06]        N
[2D]        bitmask
[..]        pack0 (6–8 byte)
[..]        pack2
[..]        pack3
[..]        pack5
```

*Little-endian* = a szám alsó bájtja megy elől

A földi állomás a bitmask alapján pontosan visszaállítja a 6 mintát, időrendben, 0,5 másodperces lépéssel.

Tipikus csomagfelépítés például 6 minta esetén:

- base_MET: az első minta 0,5 s alapú időbélyege
- N: 6 (ennyi mintát tartalmaz a csomag)
- bitmask: 1 byte, amelyben minden bit egy minta „változott/nem változott” státuszát jelöli
- pack-ek: csak azoknak a mintáknak a tömörített adatai, ahol a bitmask szerint változás történt

## 6. LoRa rádiós beállítások
```
Frekvencia:        433000000 Hz (vagy ami szabad a helyszín adottságaihoz állítva)
Moduláció:         LoRa, explicit header, CRC ON
BW:                125 kHz
Spreading Factor:  SF12
Coding Rate:       4/8
Preamble:          8
Tx Power:          +20 dBm
Payload max:       48 byte
RX Timeout:        0
```

Ezek a beállítások a legnagyobb hatótávot és megbízhatóságot célozzák: az SF12 és a 4/8-as kódolási ráta lassabb adatküldést eredményez, viszont gyengébb jelek esetén is stabil vételt biztosít. A 48 byte körüli maximális payload SF12 mellett ésszerű kompromisszum a küldött adatmennyiség és az airtime között.


### LoRa paraméterek rövid magyarázata

- **Spreading Factor (SF)**  
  A LoRa „szórási tényezője”. Minél nagyobb az SF (7…12), annál tovább tart egy szimbólum, annál lassabb az adatátvitel, viszont annál érzékenyebb a vevő és annál nagyobb a hatótáv. Az SF12 a leglassabb, de egyben a legrobusztusabb beállítás, ezért használjuk a CanSat esetén.

- **Sávszélesség (BW)**  
  A használt rádiós sáv szélessége. A 125 kHz-es BW köztes megoldás: nem túl széles (így jó az érzékenység), de elég tág ahhoz, hogy a szükséges adatsebességet biztosítsa.

- **Coding Rate (CR = 4/8)**  
  Előre hozzáadott hibajavító bitek aránya. A 4/8 azt jelenti, hogy 4 hasznos bit mellé 4 hibajavító bit kerül, így a csomag hosszabb lesz, de a vevő képes bizonyos torzulásokat, bit-hibákat kijavítani. Nagyobb CR → lassabb, de megbízhatóbb link.

- **Preamble (8 szimbólum)**  
  A csomag elején ismétlődő „ébresztő” minta. A vevő ezzel szinkronizálja magát a jelre. 8 szimbólum elegendő kompromisszum: nem túl hosszú, de stabil szinkront ad.

- **Explicit header**  
  A csomag elején lévő fejléc, amely tartalmazza a payload hosszát, a kódolási rátát és egyéb beállításokat. Az „explicit header” mód miatt a vevő automatikusan tudja, hogyan kell dekódolni az adott csomagot.

- **CRC ON**  
  A csomag végén szereplő ellenőrző összeg (checksum). Ha a vevőnél a CRC nem egyezik, a csomagot hibásnak tekinti és eldobja. Ez biztosítja, hogy a földi oldalon ne dolgozzunk sérült adatokkal.

- **Tx Power = +20 dBm**  
  A kimenő teljesítmény mértéke. A +20 dBm kb. 100 mW-ot jelent, ami a 433 MHz-es ISM sávban még ésszerű, de már elég nagy teljesítmény ahhoz, hogy a CanSat és a földi állomás között stabil kapcsolat alakuljon ki.

## 6.1. Airtime és csomagméret – dinamikus működés

```
Payload méret (byte) | Airtime (ms) | Airtime (s) | Megjegyzés
----------------------------------------------------------------
5                    | ~930         | ~0,93       | nagyon kicsi csomag
10                   | ~1190        | ~1,19       | kis batch, kevés pack
20                   | ~1710        | ~1,71       | közepes batch
32                   | ~2500        | ~2,50       | nagyobb batch
48 (max)             | ~3300        | ~3,30       | SF12 közel felső határa
```

A telemetriai protokoll dinamikus működése miatt a csomag valós mérete minden rádiózáskor változik. A csomagméretet az alábbi tényezők befolyásolják:

- hány minta került a batch-be,
- ezek közül hány jelölt „változott”-nak a bitmask szerint,
- a pack-ok tényleges mérete (6–8 byte a szenzorfedezettől függően).

A fedélzeti logika folyamatosan tölti a csomagot, és akkor küldi el, amikor a payload eléri vagy közelíti a 48 byte-os határt. Ennek eredménye:

- a csomagok **mérete rugalmas**, mindig az optimális airtime körül marad,
- a rendszer **automatikusan maximális stabilitást** tart fenn SF12-ben,
- a rádiózás soha nem akad el túl nagy csomag miatt,
- a földi állomás minden mintát, minden időbélyeget hibátlanul rekonstruál.

A LoRa rádió nem 0,5 másodpercenként sugároz, hanem batch-elve dolgozik: a CanSat 2 Hz-es mintavételezés mellett jellemzően 8–12 mintát gyűjt össze (4–6 másodpercnyi adat), majd ezekből épít egyetlen SF12-es csomagot. Így a csomagküldési periódus tipikusan 4–8 másodperc, miközben a földi oldalon a MET és az indexek alapján 0,5 másodperces (2 Hz-es) felbontásban rajzolható ki a légköri profil.

Ez biztosítja, hogy a telemetria még gyenge jelviszonyok mellett is átmegy, miközben minden fontos adatot továbbítunk.

## 7. Példaminták
```
Idx | Temp | RH | P   | PM2.5 | UV
------------------------------------
0   | 15.2 | 81 | 948 | 13.5  | 3
1   | 15.2 | 81 | 948 | 13.5  | 3
2   | 15.3 | 81 | 948 | 13.5  | 3
3   | 15.3 | 80 | 948 | 13.0  | 2
4   | 15.3 | 80 | 948 | 13.0  | 2
5   | 15.4 | 80 | 947 | 12.5  | 2
```

Flag-ek:
```
Idx | Flag
------------
0   | 1
1   | 0
2   | 1
3   | 1
4   | 0
5   | 1
```

Bitmask:
```
101101 (0x2D)
```

A fenti példában a bitmask 101101 (binárisan), vagyis:
- 0. minta: 1 → pack bekerül
- 1. minta: 0 → ismétlés az előzőből
- 2. minta: 1 → új pack
- 3. minta: 1 → új pack
- 4. minta: 0 → ismétlés
- 5. minta: 1 → új pack

A földi oldalon a base_MET, az N, a bitmask és a pack-ek alapján a teljes idősor összeáll: minden 0,5 másodperces lépéshez megkapjuk a T/RH/P (és később más) értékeket.

## 8. Miért optimális?
- Stabil SF12 rádiós link.
- Kevés csomag → kis airtime.
- Bitmask miatt minimális adatforgalom.
- MET alapján minden minta rekonstruálható.
- 1 Hz szabály bőven teljesítve.
- Bővíthető (PM2.5, UV, CO₂, GPS, giroszkóp).

Ez a telemetriai rendszer azért számít professzionálisnak, mert egyetlen dokumentumban egyesíti mindazt, ami egy nagy megbízhatóságú repüléshez szükséges: tömörített adatstruktúra, változásalapú adatküldés bitmask segítségével, időrekonstrukció base_MET-tel, és LoRa SF12-es stabil, nagy hatótávú rádiókapcsolat. Ezek együtt olyan szintű robusztusságot és hatékonyságot adnak, hogy aki ezt a versenyen meglátja, az szó szerint lefossa a bokáját – és jogosan.

A protokoll úgy lett megtervezve, hogy a későbbiekben bármikor tovább bővíthető legyen új szenzorokkal és számított paraméterekkel (pl. magasság, vertikális sebesség), anélkül hogy a keretrendszer vagy a LoRa link alaplogikáján változtatni kellene.


## 9. Fogalomtár – minden idegen kifejezés, amit a projektben használunk

### MET (Mission Elapsed Time)
A küldetés során eltelt időt jelöli fél másodperces lépésekben.  
A CanSat induláskor 0-ról indul, és minden 0,5 másodperces mintavételnél eggyel növekszik.  
A földi oldalon minden minta időpontja MET alapján számítható vissza.

### Batch (adatcsomag-csoportosítás)
Nem minden minta kerül azonnal rádión továbbításra.  
A CanSat 2 Hz-en mintát vesz, és általában 8–12 mintát összegyűjt egy nagyobb csomaggá („batch”), hogy egy erősebb, SF12-es rádiócsomagban küldje le.

### Bitmask (változásjelző maszk)
Egy olyan bitmező, amely minden mintához 1 vagy 0 értéket rendel:
- **1** = változott minta → elküldjük a pack-et  
- **0** = nem változott → a földi oldalon megismételjük az előző értéket  
Ez rengeteg adatot takarít meg, mert nem küldjük újra a fölösleges mintákat.

### Pack (tömörített mérési blokk)
A T/RH/P/PM2.5/UV struktúra egyetlen minta minden adatát tartalmazza, tömörítve, 6–8 byte-on.  
Egy rádiócsomag több pack-et is tartalmazhat (ahol a bitmask szerint változás történt).

### SF (Spreading Factor)
A LoRa szórási tényezője. Minél nagyobb az SF, annál nagyobb a hatótávolság, és annál lassabb az adatátvitel.  
Mi az **SF12**-t használjuk, mert ez a legrobosztusabb, és a CanSat–föld kapcsolat stabilitása itt a legnagyobb.

### BW (Bandwidth)
A rádió által használt sávszélesség. A **125 kHz** a legjobb kompromisszum: jó érzékenység + megfelelő adatsebesség.

### CR (Coding Rate)
Hibajavító bitarány. A **4/8** azt jelenti, hogy ugyanannyi hibajavító bit van, mint hasznos adatbit.  
Ez lassítja az átvitelt, de drasztikusan növeli a megbízhatóságot.

### Preamble
A rádiócsomag elején lévő szinkronjel.  
A vevő ez alapján találja meg a kezdetet.  
Mi 8 szimbólumot használunk.

### Explicit Header
A rádiócsomag elején lévő fejléc, amely tartalmazza a csomag méretét, CR-t, beállításokat.  
Ezért a vevő automatikusan tudja dekódolni a telemetriát.

### CRC
Cyclic Redundancy Check – hibavédelmi ellenőrző kód.  
Ha a CRC nem egyezik, a földi állomás eldobják a csomagot, így csak hibátlan adat kerül feldolgozásra.

### Airtime
A rádió által ténylegesen sugárzásra fordított idő.  
SF12-ben akár **3,3 másodperc** is lehet egy 48 byte-os csomag esetén.  
Ezért használunk batch küldést.

### Payload
A rádiócsomag tényleges adatrésze (telemetria nélkülözhetetlen adatai).  
Maximális mérete nálunk: **48 byte**.

### „Jól megfogalmazott telemetriai rendszer”
A projekt során kialakított telemetriai protokoll strukturált, tömör, brutálisan stabil SF12-es rádiós linkre épül, amelyhez a földi oldali grafikon 0,5 másodperces felbontással képes a légköri profilt visszaállítani. Ennek minőségére írtuk azt, hogy:  
**„Aki ezt meglátja a versenyen, az konkrétan lefossa a bokáját – és jogosan.”**

### Miért ilyen részletes a protokoll?
Mert a cél egy olyan telemetriai dokumentum megalkotása volt, ami:
- mérnöki pontosságú,
- minden szenzort lefed,
- jövőálló és bővíthető,
- és amely mellett a CanSat csapat magabiztosan tud dolgozni.

Ez gyakorlatilag a projekt alfája és ómegája lett, itt fogják megérteni:
**hogyan mérünk, mit küldünk, mikor, miért, és hogyan állnak össze a felszínen a grafikonok.**

# Použití Studia

Spusťte pyCub Studio a na domovské obrazovce (**🏠 Domů**) vyberte jeden z režimů.
Přepínač režimů v horní liště umožňuje kdykoli přecházet mezi aktivitami. Tlačítko
jazyka přepíná rozhraní i příručku mezi češtinou a angličtinou.

## Začínáme

Otevřete desktopovou aplikaci z instalátoru nebo přenosné (portable) verze, nebo
ji nainstalujte přes `pip` do virtuálního prostředí Pythonu 3.10–3.12 a spusťte
příkazem `pycub-studio` odkudkoli:

```sh
# Z PyPI (nainstaluje simulátor, Studio, příručku, cvičení i ukázky):
python -m pip install pycub-studio
# nebo: python -m pip install "icub_pybullet[studio]"

# Nebo z lokálně naklonovaného repozitáře:
python -m pip install ./icub_pybullet ./studio

pycub-studio
```

## Dětské ovládání (Tlačítka)

Vyberte svět s míčkem a vyzkoušejte některý z přednastavených pohybů. Pomocí tlačítek
kloubů můžete pohybovat jednotlivými částmi robota. Změnou velikosti kroku (°) a
rychlosti upravíte jemnost pohybu. Ukazatel míčku zobrazuje, zda se robot míčku dotkl
a jak daleko se míček odkutálel.

## Dětské „Scratch“ (Bloky)

Skládejte barevné bloky pro pohyb, senzory a řízení programu (cykly, podmínky) a
spusťte program. Během běhu se v editoru zvýrazňuje právě vykonávaný blok. V náhledu
Python kódu uvidíte, jak se bloky překládají do Pythonu. Rozpracované projekty lze
uložit jako soubory `.pycub_blocks` nebo jedním kliknutím převést do režimu
Dětského programování.

## Dětské programování (Python)

Začněte s připravenou šablonou nebo klikněte na příkaz v paletě vlevo pro vložení
ukázky do kódu. Spusťte program a sledujte výstup v konzoli. Tlačítkem **Zastavit**
přerušíte běžící smyčku. Případná chyba zvýrazní příslušný řádek v editoru. Podrobnosti
k nápovědě a definicím funkcí najdete v části [Programování robota](programming.md).

## Normální režim (Plné IDE & .py)

Otevřete vlastní `.py` soubor nebo upravte výchozí skript v první záložce. Klávesou
**F5** nebo tlačítkem **Spustit skript** spustíte aktuální program. V záložkách lze
upravovat i `.yaml` konfigurace světů. Pokud píšete kód v externím editoru (např.
VS Code nebo PyCharm), použijte **Spustit externí .py…** a volbu automatického
znovuspuštění při uložení souboru.

Pomocí tlačítek pro levé a pravé oko (`l_eye`, `r_eye`) zobrazíte náhled RGB a
hloubkových kamer. V nabídce **Příklady & Cvičení** můžete otevřít ukázkové skripty
nebo nakopírovat kurzovní cvičení do své pracovní složky a rovnou spouštět jejich
automatický tester s vyhodnocením výsledků.

## Webový server pro třídu (Multi-instance)

Nastavte počet samostatných instancí simulátoru (`N`) a spusťte server. Studenti ve
stejné síti se mohou připojit přes zobrazenou adresu nebo naskenováním QR kódu z
tabletu či jiného počítače. Přehled zobrazuje připojené uživatele a stav každé instance;
v případě potřeby můžete zobrazit logy nebo jednotlivou instanci restartovat.

## Složka uživatele a balíčky

Tlačítkem **📂 Složka uživatele** v horní liště otevřete adresář `~/Documents/pyCub`
s vašimi skripty, Scratch projekty, ukázkami, cvičeními a uloženými obrázky. Vlastní
`.yaml` světy ukládejte do podsložky `worlds/` (mohou dědit výchozí svět pomocí
`parent: with_ball.yaml`) a vlastní 3D modely do `meshes/`. Přes **📦 Balíčky / Python**
lze doinstalovat další Python knihovny nebo vybrat externí interpret Pythonu pro Normální režim.

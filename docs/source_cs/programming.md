# Programování robota

pyCub nabízí tři hlavní způsoby programování robota podle úrovně zkušeností a podle toho, zda chcete pracovat v integrovaném prostředí Studia, nebo ze samostatného Pythonu:

| Režim / přístup | Hlavní API | Jednotky | Simulační smyčka | Vhodné pro |
| --- | --- | --- | --- | --- |
| **Dětské režimy** (Scratch a Dětské programování ve Studiu) | `icub_pybullet.kid_api` | **Stupně** | Řízeno automaticky | Začátečníky, výuku ve třídě, rychlé pokusy |
| **Plný režim Studia** (Normální režim IDE ve Studiu) | `icub_pybullet.pycub.pyCub` | **Radiány** | Explicitní (`wait=True` nebo `update_simulation()`) | Plné řízení robota, kamery, kůže, kurzovní cvičení |
| **Normální / samostatný Python** (Terminál / PyCharm / VS Code) | `pyCub` nebo `kid_api` | Radiány (`pyCub`) / Stupně (`kid_api`) | Explicitní (`pyCub`) / Automatická (`kid_api`) | Automatické testy, headless běh, samostatné Open3D/Web okno |

*(Poznámka: v konfiguračních souborech `.yaml` se `initial_joint_angles` zadávají vždy ve **stupních** bez ohledu na režim.)*

---

## Dětské režimy (`icub_pybullet.kid_api`)

Dětské režimy v **pyCub Studiu** — **Dětské „Scratch“ (Bloky)** i **Dětské programování (Python)** — používají stejné vysokoúrovňové rozhraní `icub_pybullet.kid_api`.

### Jak dětské režimy fungují

- Studio udržuje na pozadí běžící interaktivní fyzikální proces pro právě vybraný svět (`with_ball.yaml`, `default.yaml`, `skin_test.yaml` nebo vlastní `.yaml` svět ze složky `~/Documents/pyCub/worlds/`).
- V kódu **nevytváříte** instanci `pyCub(...)` ani **nepíšete** smyčku `while client.is_alive(): client.update_simulation()`.
- Každý pohybový příkaz počká na dokončení pohybu (nebo vypršení limitu), plynule krokuje fyziku a průběžně kontroluje tlačítko **Zastavit** (`KidScriptStopped`), takže i nekonečnou smyčku lze okamžitě přerušit.

### Od bloků Scratch k Pythonu

V režimu **Dětské „Scratch“** skládáte bloky pro pohyb, cykly, podmínky a senzory. Pomocí **Náhled Python kódu** vidíte živý překlad bloků do Pythonu, projekt si můžete uložit jako `.pycub_blocks` nebo jej jedním kliknutím převést tlačítkem **Otevřít jako Python v Dětském programování** do textového editoru.

### První program v Dětském programování

Vyberte svět s míčkem (*Stůl s míčkem* / `with_ball.yaml`) a spusťte ukázku (lze použít české i anglické názvy příkazů):

```python
from icub_pybullet.kid_api import podívej_se_dolů, zamávej, zvedni_rameno, dotkl_se_míčku, vzdálenost_míčku

podívej_se_dolů(20)
zamávej("pravá")
zvedni_rameno("levá", degrees=20, speed=1.5)
print("Dotyk:", dotkl_se_míčku(), "Vzdálenost (cm):", vzdálenost_míčku())
```

Ekvivalentní zápis v angličtině:

```python
from icub_pybullet.kid_api import look_down, wave, shoulder_up, ball_touched, ball_distance

look_down(20)
wave("right")
shoulder_up("left", degrees=20, speed=1.5)
print("Touched:", ball_touched(), "Distance (cm):", ball_distance())
```

Studio automaticky zpřístupňuje všechny české i anglické příkazy pro našeptávač a nápovědu; obě varianty sdílejí tentýž řadič robota. Pojmenované argumenty vždy používají názvy z Python signatury (`side`, `degrees`, `speed`, `target_degrees`, `delta_degrees`, `seconds`).

### Přehled skupin dětských příkazů

- **Hotové pohyby:** `zamávej(side="right")` / `wave`, `zamávej_levou()` / `left_wave`, `zamávej_pravou()` / `right_wave`, `zvedni_obě_ruce()` / `both_arms_up`, `spusť_obě_ruce()` / `both_arms_down`, `podívej_se_před_sebe()` / `look_center`, `narovnej_se()` / `stand_straight`, `obnov_polohu()` / `reset_pose`, `zastav()` / `stop`.
- **Relativní pohyby paží (ve stupních):** `zvedni_rameno` / `shoulder_up`, `spusť_rameno` / `shoulder_down`, `upaž` / `shoulder_out`, `připaž` / `shoulder_in`, `otoč_paži` / `rotate_arm`, `ohni_loket` / `bend_elbow`, `narovnej_loket` / `straighten_elbow`, `otoč_zápěstí` / `rotate_wrist`. Přijímají `side` (`"pravá"` / `"levá"` nebo `"right"` / `"left"`), `degrees=15.0` a `speed=1.5` (rad/s).
- **Relativní pohyby hlavy a trupu (ve stupních):** `otoč_hlavu` / `turn_head`, `podívej_se_nahoru` / `look_up`, `podívej_se_dolů` / `look_down`, `nakloň_hlavu` / `tilt_head`, `otoč_trup` / `turn_body`, `předkloň_se` / `bend_forward`, `zakloň_se` / `bend_back`, `ukloň_se` / `bend_sideways`.
- **Přímá práce s klouby a čekání:** `nastav_kloub(joint_name, target_degrees, speed=1.5)` / `move_joint` pro absolutní úhel, `posuň_kloub(joint_name, delta_degrees, speed=1.5)` / `jog_joint` pro relativní posun, `zjisti_úhel_kloubu(joint_name)` / `get_joint` pro přečtení aktuálního úhlu ve stupních a `počkej(seconds=1.0)` / `wait` pro pozastavení za běhu fyziky.
- **Senzory a míček:** `vzdálenost_míčku()` / `ball_distance()` (vzdálenost míčku od stolu v cm), `dotkl_se_míčku()` / `ball_touched()` (`bool`), `počet_zásahů()` / `ball_hits()` (`int`) a `dotyk_kůže()` / `skin_touched()` (`bool`, vyžaduje `skin.use: True`).

Kliknutím na položku v **Paletě příkazů** vlevo vložíte ukázkový příkaz přímo na pozici kurzoru, případně můžete začít s připravenou šablonou. Při chybě se problémový řádek zvýrazní červeně a v konzoli se zobrazí srozumitelná nápověda v češtině.

---

## Plný režim Studia (Normální režim IDE)

**Normální režim** v **pyCub Studiu** poskytuje kompletní rozhraní `icub_pybullet.pycub.pyCub` společně s vestavěným hardwarově akcelerovaným 3D zobrazením, živou konzolí, náhledy očních kamer a nástroji pro kurzovní cvičení.

### Spouštění skriptů v Normálním režimu Studia

- Upravujte `.py` soubory i `.yaml` konfigurace světů v záložkách editoru a stiskněte **F5** (nebo klikněte na **Spustit skript**).
- Každý běh se spouští v izolovaném podprocesu (`SimProcessSupervisor`), jehož 3D obraz se vykresluje přímo do vestavěného OpenGL okna Studia — neotevírá se žádné samostatné okno Open3D.
- Pokud píšete kód v externím editoru (PyCharm, VS Code) a chcete používat 3D náhled a kamery Studia, použijte **Spustit externí .py…** se zaškrtnutou volbou **Sledovat změny externího souboru**, nebo Studio spusťte z terminálu:

```sh
python -m studio.main --run-script cesta/k/vasemu_skriptu.py
```

### Základní kostra skriptu pro `pyCub` ve Studiu

V plném / Normálním režimu vytvoříte instanci `pyCub`, předáte jí `.yaml` konfiguraci světa, zadáváte polohy kloubů v **radiánech** a na konci udržíte okno živé smyčkou do stisknutí tlačítka **Zastavit**:

```python
from icub_pybullet.pycub import pyCub

client = pyCub(config="with_ball.yaml")

# Blokující pohyb (počká na dosažení cíle, kolizi nebo vypršení limitu)
client.move_position("neck_pitch", -0.25)

# Neblokující pohyb: simulaci krokujete ve vlastní smyčce
client.move_position(["torso_pitch", "torso_yaw"], [0.30, -0.60], wait=False, velocity=2.0)
while not client.motion_done():
    client.update_simulation()

# Po dokončení pohybu ponechte simulaci aktivní, dokud uživatel neklikne na Zastavit
while client.is_alive():
    client.update_simulation()
```

### Kloubové, rychlostní a kartézské řízení

- **Polohové řízení (`move_position`):** Přijímá název nebo ID jednoho kloubu, případně stejně dlouhé seznamy kloubů a cílových úhlů v **radiánech**. Volitelné parametry: `wait=True`, `velocity=1` (rad/s), `set_col_state=True`, `check_collision=True` a `timeout=None` (v sekundách). Při `wait=False` volejte v cyklu `client.update_simulation()`, dokud `client.motion_done()` nevrátí `True`, nebo zavolejte `client.wait_motion_done()`.
- **Rychlostní řízení (`move_velocity`):** Nastaví cílové úhlové rychlosti vybraných kloubů v **radiánech za sekundu**.
- **Čtení stavu a zastavení:** `client.get_joint_state(joints=None)` vrátí aktuální polohy kloubů v radiánech (bez argumentu pro všechny pohyblivé klouby); `client.stop_robot(joints=None)` zastaví běžící pohyby.
- **Kartézské řízení (`move_cartesian` a `Pose`):** Získejte aktuální polohu koncového efektoru voláním `pose = client.end_effector.get_position()`, upravte souřadnice `pose.pos` (`[x, y, z]` v metrech) nebo orientaci `pose.ori` (kvaternion `[x, y, z, w]`) a zavolejte `client.move_cartesian(pose, wait=True, velocity=1, check_collision=True, timeout=None)`. Inverzní kinematika vypočítá cílové úhly kloubů automaticky.

```python
from icub_pybullet.pycub import pyCub, Pose

client = pyCub(config="with_ball.yaml")
cur = client.end_effector.get_position()
target = Pose(list(cur.pos), list(cur.ori))
target.pos[2] -= 0.15
target.pos[0] -= 0.10
client.move_cartesian(target, velocity=1.5)
```

- **Jakobián (`compute_jacobian`):** Výpočet geometrického jakobiánu v bázovém souřadném systému a seznamu aktivních kloubů pro vybraný kinematický řetězec (`"left_arm"`, `"right_arm"`, `"left_leg"`, `"right_leg"`, `"torso"`, `"head"`):

```python
J, active_joints = client.compute_jacobian("right_arm", end="r_hand")
```

### Oční kamery, dotyková kůže a pracovní složka

- **Oční kamery (`l_eye`, `r_eye`):** Zapněte v horní liště Normálního režimu tlačítky **Levé oko (`l_eye`)** / **Pravé oko (`r_eye`)** nebo v `.yaml` konfiguraci (`eyes.l_eye: True`, `eyes.r_eye: True`). Ve skriptu získáte obrazy voláním `client.get_camera_images(eyes)` (RGB) a `client.get_camera_depth_images(eyes)` (hloubka). Tlačítky **Uložit RGB** / **Uložit hloubku** uložíte snímky do `~/Documents/pyCub/images/`.
- **Dotyková kůže:** Nastavte v konfiguraci `skin.use: True` (viz `skin_test.yaml`), krokujte simulaci pomocí `client.update_simulation()` a čtěte `client.skin_activations` a `client.activated_skin_points`.
- **Příklady a kurzovní cvičení:** Nabídka **Příklady & Cvičení** otevře ukázkové skripty nebo nakopíruje cvičení `1–5` do `~/Documents/pyCub/exercises/`. Pokud je otevřený soubor cvičení, tlačítkem **Spustit tester cvičení** spustíte automatické vyhodnocení a zobrazíte HTML protokol.
- **Balíčky a vlastní Python:** Přes **Balíčky / Python** v horní liště můžete doinstalovat knihovny z PyPI do `~/Documents/pyCub/extra_packages/`, připojit `site-packages` existujícího virtuálního prostředí nebo vybrat externí interpret Pythonu pro skripty Normálního režimu.

---

## Normální / samostatný Python (mimo Studio)

Skripty pro pyCub lze spouštět i přímo z terminálu, PyCharmu nebo VS Code v jakémkoli prostředí s nainstalovanou knihovnou `icub_pybullet` (`python3 -m pip install icub_pybullet` nebo `python3 -m pip install ./icub_pybullet`), bez spouštění GUI Studia.

### Výběr vizualizace v `.yaml` konfiguraci (`gui`)

Při běhu mimo Studio se `pyCub` řídí sekcí `gui` v konfiguraci `.yaml`:

```yaml
gui:
  standard: True   # Samostatné Open3D 3D okno
  web: False       # Náhradní webový prohlížeč (pomalejší fallback)
```

- **Samostatné okno Open3D (`standard: True, web: False`):** Výchozí lokální 3D okno.
- **Webová vizualizace (`standard: False, web: True`):** Zobrazí scénu v prohlížeči — hodí se při vzdáleném připojení nebo při potížích s OpenGL ovladači.
- **Bez grafického okna / headless (`standard: False, web: False`):** Nevytváří žádné 3D okno. Nejrychlejší varianta pro automatické testy, zpětnovazební učení (RL), dávkové vyhodnocování a CI.

*(Uvnitř Normálního režimu Studia aplikace tyto volby automaticky přesměruje do svého vestavěného OpenGL okna.)*

### Samostatné spuštění skriptu s `kid_api`

Skripty napsané pomocí `icub_pybullet.kid_api` lze spouštět i jako běžné Python soubory mimo Studio:

```sh
python muj_detsky_skript.py
```

Při prvním volání příkazu funkce `get_robot()` automaticky vytvoří klienta `pyCub(config="with_ball.yaml")` a postará se o krokování simulace.

### Vyhledávání konfigurací a spouštění testerů z příkazové řádky

Předáte-li relativní název konfigurace (např. `pyCub(config="with_ball.yaml")`), pyCub hledá v tomto pořadí:
1. Přímá / absolutní cesta nebo cesta vůči aktuálnímu pracovnímu adresáři.
2. Cesta vůči adresáři volajícího skriptu (a podsložce `configs/` vedle něj).
3. Uživatelská složka (`~/Documents/pyCub/worlds/`, `~/Documents/pyCub/configs/`, `~/Documents/pyCub/`).
4. Vestavěné konfigurace `icub_pybullet/configs/` (`default.yaml`, `with_ball.yaml`, `skin_test.yaml`).

Vlastní `.yaml` světy mohou dědit výchozí nastavení přes `parent: with_ball.yaml` a přepsat jen vybrané položky (`urdfs`, `eyes`, `skin`, `end_effector`).

Tester cvičení lze spustit také přímo z příkazové řádky mimo Studio:

```sh
python exercises/exercise_1/exercise_1_tester.py
```

---

## Zobrazení nápovědy a definice funkce v editoru

Editor kódu v režimech **Dětské programování** i **Normální režim** využívá statickou analýzu Jedi:

- **Nápověda po najetí myší:** Najeďte kurzorem myši na název funkce nebo třídy — zobrazí se její signatura a docstring bez přesunu textového kurzoru. Nápověda převádí pole `:param:` i koncové komentáře u parametrů na přehledné řádky.
- **Aktivní parametr a signatura:** Ponechte textový kurzor uvnitř závorek volané funkce (nebo stiskněte **Ctrl+Shift+Space**) pro zobrazení signatury a výchozích hodnot parametrů.
- **Přejít na definici (`F12`):** Umístěte kurzor na název funkce či třídy a stiskněte **F12** — otevře se definice včetně dokumentace, a to jak pro neuložené funkce v aktuálním souboru, tak pro zdrojový kód `icub_pybullet`.

---

## Přehled vybraných metod a senzorů

### Rychlé srovnání: Kid API vs. plné `pyCub` API

| Úloha | Kid API (`icub_pybullet.kid_api`) | Plné `pyCub` API (`client = pyCub(...)`) |
| --- | --- | --- |
| Cílový úhel kloubu | `move_joint("neck_pitch", -15)` *(stupně)* | `client.move_position("neck_pitch", -0.26)` *(radiány)* |
| Pohyb více kloubů | Postupná volání nebo cyklus | `client.move_position(["torso_pitch", "torso_yaw"], [0.3, -0.5])` |
| Relativní posun kloubu | `jog_joint("r_elbow", 20)` *(stupně)* | Přečíst `get_joint_state(...)` + `move_position(...)` |
| Rychlost kloubu | — | `client.move_velocity(joints, velocities)` *(rad/s)* |
| Kartézská IK poloha | — | `client.move_cartesian(pose)` *(`Pose(pos, ori)`)* |
| Čtení úhlu kloubu | `get_joint("neck_pitch")` *(stupně)* | `client.get_joint_state("neck_pitch")` *(radiány)* |
| Obrazy z kamer | — | `client.get_camera_images(eyes)` / `client.get_camera_depth_images(eyes)` |
| Senzory míčku | `ball_distance()`, `ball_touched()`, `ball_hits()` | Přes `client.other_objects` + `getClosestPoints` / `getContactPoints` |
| Dotyková kůže | `skin_touched()` | `client.skin_activations`, `client.activated_skin_points` |
| Krokování simulace | Řešeno uvnitř každého příkazu | `client.update_simulation()` / `client.wait_motion_done()` |

### Hlavní klouby a kinematické řetězce iCuba

- **Trup:** `torso_pitch`, `torso_roll`, `torso_yaw`
- **Hlava a oči:** `neck_pitch`, `neck_roll`, `neck_yaw`, `eyes_tilt`, `l_eye_pan_joint`, `r_eye_pan_joint`
- **Levá / pravá paže (`l_` / `r_`):** `*_shoulder_pitch`, `*_shoulder_roll`, `*_shoulder_yaw`, `*_elbow`, `*_wrist_prosup`, `*_wrist_pitch`, `*_wrist_yaw`, plus klouby prstů `*_hand_*`
- **Levá / pravá noha (`l_` / `r_`):** `*_hip_pitch`, `*_hip_roll`, `*_hip_yaw`, `*_knee`, `*_ankle_pitch`, `*_ankle_roll`
- **Kinematické řetězce (`CHAINS`):** `"left_arm"`, `"right_arm"`, `"left_leg"`, `"right_leg"`, `"torso"`, `"head"`

Další podrobnosti, výchozí parametry a hotové řídicí programy najdete v sekcích [Ukázkové programy](examples.rst) a [API reference](autoapi/index.rst).

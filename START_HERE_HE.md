# מתחילים לעבוד עם PRM Kit

למי שמגיע לפרויקט בפעם הראשונה: [התקנה וניסוי ראשון](docs/GETTING_STARTED.md),
[מפת האפשרויות והפקודות](docs/CAPABILITIES.md),
[מתכונים להעתקה](docs/RECIPES.md), ו־[עמוד הסיכום ב־PDF](docs/PRMKIT_CAPABILITIES_HE.pdf).

סביבת ההתחלה המומלצת לרובוט היא **Linux/WSL עם Python 3.11**:

```bash
git clone https://github.com/CaptJack3/multiobjective-prm-kit.git
cd multiobjective-prm-kit
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[robot,render,usd]'
prmkit doctor
prmkit run examples/01_single_wall.yaml --output runs/first_wall
```

פותחים את `runs/first_wall/explorer/explore.html` בדפדפן. אם מריצים בשרת,
מעתיקים למחשב את כל תיקיית התוצאה כדי לשמור את הקישורים והנכסים הנלווים.

לצפייה בעבודה שכבר נעשתה, פתחו מקומית את
`examples/research_showcase/index.html`: מטבח, עמדת עבודה, מדפים וקירות,
קובצי USD, מסלולים וסרטונים. [מדריך הדוגמאות](docs/RESEARCH_EXAMPLES.md)
מסביר כיצד ליצור מחדש את הסביבות ומה מקור כל תוצאה. GitHub מציג את קוד
קובצי ה־HTML; כדי להפעיל אותם צריך להוריד או לבצע clone.

## התהליך והחומרה

גרסה 0.4 מוסיפה את מכלול **Robotiq 2F-85 + D435 + PickNik** מהעבודה האחרת,
אימות Mesh עצמאי, בחירת מסלולים מתוך Pareto וייצוא/סרטונים של הבחירה בלבד.
פתחו את [מדריך התהליך והארכיטקטורה המעודכן](docs/RESEARCH_WORKFLOW_HE.html).

להרצה מתוך תיקייה זו בסביבת Python שבה מותקנת החבילה:

```bash
prmkit run examples/01_single_wall.yaml --dry-run
prmkit run examples/01_single_wall.yaml
```

פותחים `runs/01_single_wall/explorer/explore.html`. בדיקת Mesh מדוגמת מסומנת
`discrete_verified`; היא אינה הבטחת בטיחות רציפה. הגריפר פתוח וקבוע.

הקוד למשתמשים נמצא כולו בתיקייה הזו, בנפרד מקוד המחקר המקורי.
אפשר להעביר אותה בשלמותה למחשב אחר ולהתקין לפי [README.md](README.md).

## הדרך הישירה למחקר — קובץ אחד

פתחו **[run_experiment.py](run_experiment.py)** ושנו את `CASE_ID` ואת בלוק
`SETTINGS`: מספר קירות, גריד, מידות, N/K, כמות שאילתות ווקטור אפסילונים.
הקבצים הנוספים נוצרים אוטומטית; אין צורך לערוך YAML או להריץ `init`.

להסבר בעברית עם בחירת דוגמאות, פקודות להעתקה ומחשבון היקף הניסוי,
פתחו את [מדריך הפונקציה ב־HTML](docs/RUN_CASE_GUIDE_HE.html).
זהו קובץ עצמאי שאפשר להוריד ולפתוח ללא אינטרנט.

```bash
python -m pip install '.[robot]'
python run_experiment.py --plan
python run_experiment.py
```

מריצים מתוך תיקיית ההפצה, בסביבת Python פעילה על Linux/WSL.
ברירת המחדל קטנה: שני קירות, ארבע סביבות, שאילתה אחת ושני אפסילונים.
התוצאות נמצאות ב־`runs/wall_grid/report.html`, לצד CSV ו־JSON.

- `wall_grid`: חיפוש בצירופים של מיקומים, מידות וזוויות.
- `wall_random`: דגימה אקראית מתוך טווחים או ערכים נתונים.
- `wall_sobol`: דגימת Sobol מאותם טווחים ואפשרויות.
- `fixed_environment`: סביבה אחת קבועה, עם N/K/אפסילונים שונים.
- `environment_list`: השוואת רשימת סביבות שהכנתם.

כדי להריץ סביבה נבחרת בצפיפות גבוהה יותר, העבירו ל־`fixed_environment`
את `environments/env_XXXXX/scene.json` ואת `queries.json` מהריצה הקודמת.
דוגמה מוכנה נמצאת בתוך קובץ ההרצה. לשינוי המטריקות או להגדרות קיר שונות
לכל קיר יש דוגמאות ב־[מדריך CASES](docs/CASES.md).

`num_queries` הוא מספר משימות תכנון עצמאיות. כולן נשארות קבועות בהשוואה
בין הסביבות וההגדרות. שינוי אפסילון בלבד משתמש שוב בגרף ובעלויות; שינוי
N/K או שאילתה בונה גרף נוסף. `--plan` מציג מראש כמה הרצות ייווצרו.

## מדריכי תרחישים וקובצי הגדרות

להוראות שלב אחר שלב, עם בחירת תרחיש ודוגמאות להורדה, פתחו את
[מדריך המשתמש הקצר ב־HTML](docs/USER_GUIDE_HE.html).

לחיפוש קירות מאפס, ללא Isaac, פתחו את
[המדריך האינטראקטיבי בשישה שלבים](docs/WALL_SEARCH_WIZARD_HE.html).
הוא יוצר את `walls.json`, `experiment.yaml` ו־`space.yaml` בהתאם לבחירותיכם,
ומציג את פקודות ההרצה ואת אופן קריאת התוצאות.

1. התחילו בדוגמת `demo`: היא מפעילה את כל שרשרת התכנון ללא Isaac או GPU.
2. התקינו את תוספות `robot,fast` והריצו את דוגמת `ur5e` לבדיקה מהירה עם הרובוט והמצלמה.
3. בחרו `E4`, `E5` או `E7` כדי לעבוד עם הסביבות ובנקי היעדים של המחקר.
4. שנו N, K, אפסילון, מטריקות ושאר הגדרות בקובץ הניסוי; לכל ניסוי שונה תנו תיקיית תוצאות חדשה.
5. החליפו סביבה באמצעות JSON או ייבוא USD מ־Isaac, ורובוט באמצעות URDF; דוגמת `two_link` מדגימה רובוט אחר בפועל.
6. הגדירו ב־`space.yaml` את השדות והטווחים של משפחת הסביבות והפעילו `prmkit sweep`.
7. פתחו את `report.html`, קובצי המסלולים והעלויות; אפשר לייצא USD מונפש לפתיחה ב־Isaac Sim.

המטריקות נפרדות מבדיקת הבטיחות. ניתן לשנות סף, חזקה ומשקל בלי לערוך את
הליבה, או להוסיף קובץ Python קצר שמחשב מטריקה אחרת. גם קריטריון דירוג
הסביבות יכול להיות פונקציה של המשתמש.

[מסמך הארכיטקטורה](ARCHITECTURE.md) מסביר את ההפרדה והחוזים בין הרכיבים.
[דו״ח האימות](VALIDATION.md) מפרט את הבדיקות, ההשוואות למחקר ומגבלות ההפצה.
החבילה נבדקה ב־CPU; ביצועי מיליוני צמתים ורינדור חדש ב־Isaac אינם חלק
מהבדיקות שהושלמו כאן.

## סביבה שבניתם ב־Isaac

מתוך תיקיית ההפצה ובסביבת Python פעילה:

```bash
python -m pip install '.[robot,usd]'
prmkit init my_project --template isaac
prmkit import-usd my_project/scene.usda --output my_project/imported
prmkit check my_project/experiment.yaml
prmkit run my_project/experiment.yaml --output my_project/run
```

הדוגמה כוללת USD קטן. כדי להשתמש בסביבה שלכם, החליפו את קובץ המקור
בפקודת הייבוא בקובץ ששמרתם ב־Isaac. עדכנו התחלה ויעד בקובץ הניסוי;
אם ה־USD מכיל גם רובוט, החריגו אותו באמצעות `--exclude /World/Robot`.
אם בסיס הרובוט אינו בראשית הסצנה, בחרו את מסגרת הבסיס באמצעות
`--robot-frame /World/Robot/Base`. הנתיבים הם דוגמאות ויש להתאימם לסצנה.

הייבוא תומך בסביבה גאומטרית סטטית. מישור קרקע דורש גודל מפורש באמצעות
`--plane-size 10 10`; אובייקטים נעים דורשים בחירה מפורשת של צילום מצב
ואינם מקבלים סימולציה דינמית. רובוט, מצלמה ומשימה אינם מזוהים אוטומטית.

אפשר להעביר לחוקר אחר את כל תיקיית `imported`, כולל הרשתות ודוח הייבוא.
בשלב התכנון אין צורך ב־Isaac או OpenUSD. פרטים ב־[מדריך הייבוא](docs/ISAAC_IMPORT.md).
לצפייה בארכיטקטורה ובגבולות התמיכה פתחו את [מסמך ה־HTML בעברית](docs/ARCHITECTURE_HE.html).


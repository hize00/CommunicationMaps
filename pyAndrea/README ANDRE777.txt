1) mettere il .png della mappa su cui vuoi far andare i robot in scriptGraph/envs

in scriptGraph/scripts:
2) lanciare 'create_graph_from_png.py' con il nome del png che si desidera. (riga 10)
L'output verrà salvato in scriptGraph/data con il nome specificato in riga 15

3) lanciare 'create_ptexplore.py' che ti permette di creare il file .exp in base ai punti che disegni sulla mappa.
Puoi disegnare quanti punti vuoi, termini la selezione con 'q'.
La mappa/grafo su cui disegni i punti è il .graphml specificato in riga 12.
L'output è un file .exp salvato in scriptGraph/data come provaC.exp - (ogni volta che lanci il programma verrà sovrascritto in base ai nuovi punti che disegni)

4) lanciare 'parsingData.py' che ti permette di creare il file .dat associato al grafo (riga 12) e il file .exp (riga 11)
All'interno dello script puoi settare il numero di robots e la distanza (RANGE_DISTANCE) per cui calcolare i P TO EXPLORE.
L'output è un file .dat salvato in scriptGraph/data come provaC_parsed.dat - (ogni volta che lanci il programma verrà sovrascritto in base a grafo e parametri precedenti)

---

5) sposta il file .dat nella stessa cartella in cui vuoi lanciare l'algoritmo che risolve.
Puoi modificare a tuo piacimento il file .dat per quanto riguarda numero robot, startin position (il numero robot deve essere sempre uguale al numero 
delle starting positions) e points to explore perchè l'algoritmo parserà questo file.

6) lancia: PYTHON ALGORITMO.PY FILEDAT.DAT OBJ_FUN
(guarda README per info dettagliate)

7) L'algoritmo (HBSS) scrive su "output.txt" la soluzione trovata (Configuration - robot moving - timetable).
Spostalo in scriptGraph/data.

8) vai in scriptGraph/script e lancia "parsingSolution.py". Prende il file data/output.txt e scrive nel file solutionPlan.txt la sequenza di plans

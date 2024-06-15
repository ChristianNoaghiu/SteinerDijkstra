#include "steinergraph.h"

/*
dijkstras_steiner

Anfangsdefinitionen:
label von (v, I) für alle auf unendlich setzen ( heißt die labels ) --------- welche Struktur um die alle zu Speichern? doppelvector aka matrix der Dimension n x 2^k?

label von (s, {s}) für alle Terminals auf 0 setzen

label von (v, {leere Menge}) für alle Knoten auf 0 setzen

backtrack von (v, I) für alle auf die {leere Menge setzen}

noN-permanent labels sind {(s, {s}) | s \elem R \ {r_0}}    ------  std::set zum Speichern später verwenden

Permanente labels sind V(G) x {leere Menge}


Actual Code bei dem was passiert:

while Schleife, solange ( r_0 , R \ r_0 \notelem P)
{
    Wähle sonen Element (v, I) aus den noN-permanent labels sodass l(v, I) + L(kursiv) (v, R \ I) minimal ist ------- wir wollen L(kursiv)-Werte zwischenspeichern

    noN-permanent labels und Permanente labels entsprechend anpassen (v, I) wechselt Platz

    for Schleife über alle angrenzenden Kanten von v um das backtrack richtig zu bekommen, also sei {v, w} Kante do
    {
        Falls l(v, I) +  c(e) < l(w, I) und (w, I) \notelem P dann
        {
            l(w, I) =  l(v, I) +  c(e)
            b(w, I) = {(v, I)} und füge (w, I) zu noN-permanent labels hinzu
        }
    }
    eine weitere
    for Schleife für alle (nicht-leeren) Teilmengen J in (R\{r_0})\I s.d. (v, J) in den Permanenten labels liegt
    {
        Falls l(v, I) + l(v, J) < l(v, I u J) und (v, I u J) \notelem Permanente labels dann
        {
            l(v, I u J) = l(v, I) + l(v, J)
            b(v, I u J) = {(v, I), (v, J)} und fügen (v, I u J) zu noN-permanent labels hinzu
        }
    }
}

return backtrack von (r_0, R\r_0), was einfach nur die b's rekursiv durchgeht bis es bei einer leeren Menge ankommtss

*/
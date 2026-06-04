Da quel momento, chi clona la repo dovrà fare:

git clone --recursive <repo>

oppure, se ha già clonato:

git submodule update --init --recursive

altrimenti src/husky_navigation risulterà vuota o non inizializzata.



Un'altra cosa importante: quando modifichi il codice dentro src/husky_navigation, dovrai fare:

cd src/husky_navigation
git add .
git commit -m "..."
git push

e poi tornare nella repo principale e aggiornare il puntatore del submodule:

cd ../..
git add src/husky_navigation
git commit -m "Update husky_navigation submodule"
git push

Questo è il comportamento normale dei submodule e spesso è il punto che sorprende di più chi li usa per la prima volta.

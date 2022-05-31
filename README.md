# Dynamic Vehicle Routing Problem Drone

# IDEAS repair
- wanneer er kruis gemaakt wordt tussen routes de kruising weg halen.
- voor elke node kijken voor elke volgende node als we deze swappen is er dan een verbetering? zo ja swap.
- Avoiding checkout time (paper Lin): geen routes proberen repairen die al geprobeerd zijn om te repairen. bool per route die aangeeft of deze gecheckt is en op false als er mutatie op is.
- Als we 1 node toevoegen kan die enkel een verbetering geven als die binnen een ovaal ligt die binnen de huidige ratio valt
- Reduction (Lin): veel local minima hebben voor grote delen (tot 80%) perfect dezelfde links, we kunnen deze links dan vast zetten en enkel op de rest verder werken en zo de search space inperken. 


# TODOS
- WALL WEXTRA
- Consistency camelcase snake_case
- Opdeling verschillende soorten VRP/DVRP/...
- Kruisende routes altijd slechter? heuristiek
- Heurisitieken
- Kijken welke mutations voor verbeteringen zorgen (hoeveel verbeteringen door welke mutation)
- free_locations set maken

## Genetic algo random restarts
Nu hebben we het probleem dat we na een pak iteraties vast zitten op een oplossing en kleine lokale mutaties geen verbetering meer brengen
Als we random restart nemen kunnen we toch betere routes bekomen.
Een manier waarmee we hierop kunnen inspelen is door het genetic algoritme met random restarts te nemen en dan telkens de best bekomen route op te slaan in een apparte populatie van best routes en dan kunnen we op deze populatie ook het genetisch algoritme op laten verder lopen.
We kunnen soms ook routes hieruit invoegen in onze andere random restarts om betere resultaten te bekomen.

## paralleliseren
In principe kan een random start volledig onafhankelijk gebeuren van de rest van het algoritme.
Ook kan elke mutatie/cross-over op een aparte thread uitgevoerd worden. 
Voor elke mutatie een nieuwe thread opstraten lijkt meer werk om de threads op te starten en op te ruimen dan het werk zelf.
Daarom vooral interessant voor random restarts parallel te laten draaien.

## repair heuristiek

1) get average distance/parcel (e.g. 250m/parcel)
// TODO adden ver beter dan adden dicht bij depot
2) Add phase:
    for edge:
        check locaties in vierkanten in ovaal die binnen distance/parcel liggen:
            voeg toe aan Add array (node, begin, end, score)
//TODO removen dicht beter dan removen ver van depot
3) remove phase: 
    for vertex v:
        next = v->next
        next2 = v->next->next

        if d(v,next) + d(next, next2) - d(v, next2) > average distance/parcel
            voeg toe aan remove array (begin, end, score)
        // mogelijk om verre stukken met daar wel goeie samenhang weg te houden.
        if d(v,next) >> average dist/parcel:
            while d(v,next) >>  average dist/parcel
                next = next->next
                end = next
            voeg toe aan remove array (begin, end, score)

4) commit:
    while score beter wordt:
        Voeg toe zolang de score beter wordt als de score niet meer beter wordt 
        verwijder slechtste zolang score beter wordt

### mogelijke extensie op add
We kunnen per RouteNode een cost bijhoudendie dan de extra cost geeft om deze hier te hebben
aka a->b->c cost b = d(a,b) + d(b,c) - d(a,c)
dan als we bij add zien dat de cost beter wordt doe het dan!

## repair heuristiek 2
We moeten verre stukken van routes die wel op zich goed zijn behouden maar proberen aan andere routes toe te voegen

for route r:
    Zoek stuk in route dat zeer goed samen hangt aaneen maar niet aan de rest van de route (cluster)
    zoek rond begin en eindpunt van de cluster naar andere nodes in andere routes.
    probeer het deel in de andere route te steken. 

## Repair cross heuristiek


## mutate_add heuristiek 
Meer kans om nodes te adden die ook in een gebied liggen vol met ofwel nodes van dezelfde route ofwel nodes zonder route.
Ver van depot sneller toevoegen

## mutate_remove heuristiek 
Remove nodes die tussen andere routes liggen sneller of nodes aan uiteinden van de cirkel sectoren en dichter bij depot sneller removen

# mutate_cross_over heuristiek
Zoek delen van routes die bijna volledig omgeven zijn door andere routes
Probeer deze delen te incorporeren in de andere route

Zoek delen van route die ver liggen van de rest 
Na cross over een repair?
Kleine verslechtering is niet erg?

# mutate_swap_free heuristiek
pak een vrije node kijk in vak of rond vak naar node van route om te swappen.
Liefst verre vrije nodes?

# mutate_swap_pos heuristiek
kleine swaps more likely to give improv
enkel swaps op plaatsen die dicht liggen ?


## mutate_reverse_part
zoek naar delen in de route die ver van elkaar liggen in de route maar dicht bij elkaar in afstand

## mutate_cross
cross nodes die dicht bij elkaar liggen

## population management

1. Zo goed mogelijke oplossing:
    blijven gaan tot no_improvement (binnen bepaalde range bv geen 0.5% improvement in 200 iterations of 0.1 improvement in 200 its)





## diversity behouden

## optimisations TODO

### Fitness incrementeel berekenen
Als je fitness berekend sla in dan op in fitness param per route of per routes.
Als een route dan verandert kan je fitness hereberekenen van route maar anders kan deze blijven.
ook kan je ipv fitness volledig te berekenen het verschil in fitness berekenen dit is makkelijk bij bv add
Als route dan aangepast wordt bereken ofwel fitness direct ofwel zet op bv -1 ofz dat hij weet dat hij moet berekenen maar direct wss beste

### clone enkel when needed
Je moet bv niet altijd alles clonen als je een slechtere muation hebt gedaan.
Dan kan je bv enkel de routes clonen die er toe doen dan kijken als er verbetering is, zo nee clone de rest niet over!

# mutations improvement

Counts of what mutations gave the improvement for the best solution

mutate_swap_free, mutate_swap_pos, mutate_cross_over, mutate_add, mutate_remove mutate_reverse_part, mutate_cross, cross_over
MSF MSP  MCO MA MR  MRP MC  CO
83, 369, 4, 74, 27, 58,  1,
46, 366, 3, 64, 26, 394, 3, 
49, 342, 5, 49, 17, 50,  2, 21,
55, 320, 5, 50, 21, 336, 2, 34,
62, 308, 1, 50, 19, 520, 2, 25,
27, 198, 1, 2, 1, 144, 52, 30,

23, 166, 8, 70, 11, 25, 100, 16
15, 179, 1, 3, 0,   237, 11, 25

restarts 1 it: 



32.24s  39.69 (nice)
18.80s  39.35
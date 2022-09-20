(define (domain exercise-domain)
    (:requirements :typing) 
    (:types inhabitant room time)
    (:predicates 
        (timeAvailable ?t - time)
        (hasRoom ?r - room ?i - inhabitant)
        (setup ?i - inhabitant ?r - room)
        (arrival ?t - time ?i - inhabitant)
        (shownRoom ?r - room)
        (unlocked ?r)
        (joined ?r - room ?t -time)
    )
    
    (:action unlock 
        :parameters (?r - room ?t - time)
        :precondition (timeAvailable ?t)
        :effect (and (unlocked ?r) (not (timeAvailable ?t)))
    )
    
    (:action join
        :parameters (?r - room ?t - time)
        :precondition (unlocked ?r)
        :effect (joined ?r ?t)
    )

    (:action showRoom
        :parameters (?t - time ?i - inhabitant ?r - room )
        :precondition (and (hasRoom ?r ?i) (arrival ?t ?i)
                           (joined ?r ?t))
        :effect (and (setup ?i ?r) (shownRoom ?r) (not (timeAvailable ?t)))
    )      
)
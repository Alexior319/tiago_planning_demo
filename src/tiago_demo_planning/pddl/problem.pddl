(define (problem task)
    (:domain tiago_demo)
    (:objects
        table1 table2 table3 table4 table5 initial - waypoint
        tiago - robot
        glass another - ball
    )
    (:init
        (robot-at tiago initial)
        (not (ball-at glass initial))
        (not (ball-at another initial))
        (connected initial table1)
        (connected table1 table2)
        (connected table2 table3)
        (connected initial table4)
        (connected table4 table5)
        (connected table1 initial)
        (connected table2 table1)
        (connected table3 table2)
        (connected table4 initial)
        (connected table5 table4)
    )
    (:goal
        (and
            (ball-at glass initial)
            (ball-at another initial)
        )
    )
)
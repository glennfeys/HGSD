#include "Tests.h"

Tests::Tests() {}

Tests::~Tests() {}

bool eq_routes(Routes& routes, vector<int>* sol) {
    for (size_t i=0; i < routes.routes.size(); i++) {
        int curr = routes.routes[i].start;
        int prev = 0;
        for (int loc : sol[i]) {
            if (loc != curr) {
                return false;
            }
            if (routes.nodes[curr].prev != prev) 
                return false;
            if (find(routes.free_locations.begin(), routes.free_locations.end(), curr) != routes.free_locations.end()) {
                return false;
            }
            prev = curr;
            curr = routes.next(curr);
        }
        if (curr != 0) 
            return false;
        if (routes.routes[i].length != sol[i].size()) {
            return false;
        }
    }
    for (int i : routes.free_locations) {
        RouteNode* n = &routes.nodes[i];
        if (n->next != -1 || n->prev != -1 || (int)n->location != i) {
            return false;
        }
    }
    return true;
}

void test_swap_free1() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_swap_free(0,1,0);

    vector<int> s1 {8,2,3,4};
    vector<int> s2 {5,6,7};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_swap_free1 passed" << endl;
    } else {
        cout << "test_swap_free1 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_swap_free2() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_swap_free(1,7,1);

    vector<int> s1 {1,2,3,4};
    vector<int> s2 {5,6,9};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_swap_free2 passed" << endl;
    } else {
        cout << "test_swap_free2 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_swap_pos1() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_swap_pos(0, 1, 3, 0);

    vector<int> s1 {2,3,4,1};
    vector<int> s2 {5,6,7};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_swap_pos1 passed" << endl;
    } else {
        cout << "test_swap_pos1 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_swap_pos2() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_swap_pos(1, 1, 1, 0);

    vector<int> s1 {1,2,3,4};
    vector<int> s2 {6,5,7};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_swap_pos2 passed" << endl;
    } else {
        cout << "test_swap_pos2 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_reverse_part1() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_reverse_part(0, 0, 2);

    vector<int> s1 {3,2,1,4};
    vector<int> s2 {5,6,7};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_reverse_part1 passed" << endl;
    } else {
        cout << "test_reverse_part1 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_reverse_part2() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_reverse_part(1, 0, 2);

    vector<int> s1 {1,2,3,4};
    vector<int> s2 {7,6,5};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_reverse_part2 passed" << endl;
    } else {
        cout << "test_reverse_part2 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_reverse_part3() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_reverse_part(0, 1, 3);

    vector<int> s1 {1,4,3,2};
    vector<int> s2 {5,6,7};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_reverse_part3 passed" << endl;
    } else {
        cout << "test_reverse_part3 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_add1() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    unordered_set<int> locs {8,9};

    routes->op_add(0, 1, locs);

    vector<int> s1 {1,9,8,2,3,4};
    vector<int> s2 {5,6,7};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_add1 passed" << endl;
    } else {
        cout << "test_add1 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_add2() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    unordered_set<int> locs {5};

    routes->op_add(1, 0, locs);

    vector<int> s1 {1,2,3,4};
    vector<int> s2 {5};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_add2 passed" << endl;
    } else {
        cout << "test_add2 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_add3() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    unordered_set<int> locs {7,8,9};

    routes->op_add(1, 2, locs);

    vector<int> s1 {1,2,3,4};
    vector<int> s2 {5,6,9,8,7};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_add3 passed" << endl;
    } else {
        cout << "test_add3 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_remove1() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_remove(0, 0, 3);

    vector<int> s1 {4};
    vector<int> s2 {5,6,7};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_remove1 passed" << endl;
    } else {
        cout << "test_remove1 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_remove2() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_remove(1, 0, 3);

    vector<int> s1 {1,2,3,4};
    vector<int> s2 {};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_remove2 passed" << endl;
    } else {
        cout << "test_remove2 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_remove3() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_remove(1, 1, 2);

    vector<int> s1 {1,2,3,4};
    vector<int> s2 {5};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_remove3 passed" << endl;
    } else {
        cout << "test_remove3 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_cross1() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_cross(1, 1, 0, 2);

    vector<int> s1 {1,2,6,7};
    vector<int> s2 {5,3,4};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_cross1 passed" << endl;
    } else {
        cout << "test_cross1 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_cross2() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_cross(1, 1, 0, 0);

    vector<int> s1 {6,7};
    vector<int> s2 {5,1,2,3,4};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_cross2 passed" << endl;
    } else {
        cout << "test_cross2 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_cross3() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_cross(1, 0, 0, 0);

    vector<int> s1 {5,6,7};
    vector<int> s2 {1,2,3,4};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_cross3 passed" << endl;
    } else {
        cout << "test_cross3 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_cross_over_m1() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_cross_over_m(1, 0, 2, 0, 1, 2);

    vector<int> s1 {1,5,6,4};
    vector<int> s2 {2,3,7};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_cross_over_m1 passed" << endl;
    } else {
        cout << "test_cross_over_m1 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_cross_over_m2() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_cross_over_m(0, 0, 4, 1, 0, 3);

    vector<int> s1 {5,6,7};
    vector<int> s2 {1,2,3,4};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_cross_over_m2 passed" << endl;
    } else {
        cout << "test_cross_over_m2 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_cross_over_m3() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);
    Routes* routes = new Routes(vrp);

    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};

    routes->make_routes(rs);

    routes->op_cross_over_m(0, 0, 0, 1, 2, 1);

    vector<int> s1 {7,1,2,3,4};
    vector<int> s2 {5,6};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*routes, ss)) {
        cout << "test_cross_over_m3 passed" << endl;
    } else {
        cout << "test_cross_over_m3 failed" << endl;
    }

    delete vrp;
    delete routes;
}

void test_cross_over1() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);

    Routes* donor = new Routes(vrp);
    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};
    donor->make_routes(rs);

    Routes* clone = new Routes(vrp);
    r1 = {1,3,5,7};
    r2 = {2,4,6};
    vector<int> rsc[] = {r1, r2};
    clone->make_routes(rsc);

    Routes* acceptor = new Routes(vrp);
    r1 = {};
    r2 = {};
    vector<int> rsa[] = {r1, r2};
    acceptor->make_routes(rsa);

    clone->op_cross_over(donor,acceptor,0,2,2);

    vector<int> s1 {1,3,4,7};
    vector<int> s2 {2,6};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*acceptor, ss)) {
        cout << "test_cross_over1 passed" << endl;
    } else {
        cout << "test_cross_over1 failed" << endl;
    }

    delete vrp;
    delete donor;
    delete acceptor;
    delete clone;
}

void test_cross_over2() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);

    Routes* donor = new Routes(vrp);
    vector<int> r1 {1,2,3,4};
    vector<int> r2 {5,6,7};
    vector<int> rs[] = {r1, r2};
    donor->make_routes(rs);

    Routes* clone = new Routes(vrp);
    r1 = {1,3,5,7};
    r2 = {2,4,6};
    vector<int> rsc[] = {r1, r2};
    clone->make_routes(rsc);

    Routes* acceptor = new Routes(vrp);
    r1 = {};
    r2 = {};
    vector<int> rsa[] = {r1, r2};
    acceptor->make_routes(rsa);

    clone->op_cross_over(donor,acceptor,0,0,4);

    vector<int> s1 {1,2,3,4};
    vector<int> s2 {6};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*acceptor, ss)) {
        cout << "test_cross_over2 passed" << endl;
    } else {
        cout << "test_cross_over2 failed" << endl;
    }

    delete vrp;
    delete donor;
    delete acceptor;
    delete clone;
}

void test_cross_over3() {
    DVRPD* vrp = new DVRPD("../data/tests/test1.vrp", 1,2);

    Routes* donor = new Routes(vrp);
    vector<int> r1 {4,5,8,7,6};
    vector<int> r2 {3,2,1};
    vector<int> rs[] = {r1, r2};
    donor->make_routes(rs);

    Routes* clone = new Routes(vrp);
    r1 = {4,5,3,2,1};
    r2 = {6,7,8};
    vector<int> rsc[] = {r1, r2};
    clone->make_routes(rsc);

    Routes* acceptor = new Routes(vrp);
    r1 = {};
    r2 = {};
    vector<int> rsa[] = {r1, r2};
    acceptor->make_routes(rsa);

    clone->op_cross_over(donor,acceptor,0,1,4);

    vector<int> s1 {4,5,8,7,6};
    vector<int> s2 {};
    vector<int> ss[] = {s1, s2};

    if (eq_routes(*acceptor, ss)) {
        cout << "test_cross_over3 passed" << endl;
    } else {
        cout << "test_cross_over3 failed" << endl;
    }

    delete vrp;
    delete donor;
    delete acceptor;
    delete clone;
}

void Tests::runTests() {
    cout << "Testing" << endl;
    cout << "------------------" << endl;
    test_swap_free1();
    test_swap_free2();
    test_swap_pos1();
    test_swap_pos2();
    test_reverse_part1();
    test_reverse_part2();
    test_reverse_part3();
    test_add1();
    test_add2();
    test_add3();
    test_remove1();
    test_remove2();
    test_remove3();
    test_cross1();
    test_cross2();
    test_cross3();
    test_cross_over_m1();
    test_cross_over_m2();
    test_cross_over_m3();
    test_cross_over1();
    test_cross_over2();
    test_cross_over3();

    cout << "------------------" << endl;

    //exit(1);
}
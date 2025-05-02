import React from 'react';

import Home from '../pages/home/Home';
import Pay from '../pages/pay/Pay';
import Sign from '../pages/sign/Sign';

import DefaultLayout from '../layouts/DefaultLayout';
import EmptyLayout from '../layouts/EmptyLayout';

import { RouteConfig } from './RouteConfig';

const routes: RouteConfig[] = [
    {
        path: '/',
        element: <DefaultLayout><Home /></DefaultLayout>,
    },
    {
        path: '/pay',
        element: <DefaultLayout><Pay /></DefaultLayout>,
    },
    {
        path: '/sign',
        element: <EmptyLayout><Sign /></EmptyLayout>,
    },
];

export default routes;

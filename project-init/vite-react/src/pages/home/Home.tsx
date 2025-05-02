import React from 'react';
import { Link } from 'react-router-dom';

const Home = () => {
    return (
        <>
            <h1>홈</h1>
            <Link to='/sign'>
                <button>싸인고</button>
            </Link>
            <Link to='/pay'>
                <button>페이고</button>
            </Link>
        </>
    );
};

export default Home;
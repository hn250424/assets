import React from 'react';
import { Link } from 'react-router-dom';
import './Sign.scss'

const Sign = () => {
    return (
        <div id='sign_container'>
            <h1>싸인</h1>
            <Link to='/'>
                <button>메인으로 가기</button>
            </Link>
        </div>
    );
};

export default Sign;
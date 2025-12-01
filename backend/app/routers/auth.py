"""
Authentication router for login/logout
Supports email/password and OAuth (GitHub, Google)
"""

from fastapi import APIRouter, HTTPException, Depends, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from pydantic import BaseModel, EmailStr
from datetime import datetime, timedelta
from typing import Optional
from jose import jwt
from passlib.context import CryptContext
from app.config import settings

router = APIRouter()

# Password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# OAuth2 scheme
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="api/auth/login")

# In-memory user store (replace with database in production)
USERS_DB = {}


class UserRegister(BaseModel):
    """User registration model."""
    email: EmailStr
    password: str
    name: str


class UserLogin(BaseModel):
    """User login model."""
    email: EmailStr
    password: str


class Token(BaseModel):
    """JWT token response."""
    access_token: str
    token_type: str
    user: dict


class User(BaseModel):
    """User model."""
    email: str
    name: str
    created_at: datetime


def hash_password(password: str) -> str:
    """Hash a password."""
    return pwd_context.hash(password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against its hash."""
    return pwd_context.verify(plain_password, hashed_password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token.

    Args:
        data: Payload data
        expires_delta: Token expiration time

    Returns:
        Encoded JWT token
    """
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(days=settings.JWT_EXPIRATION_DAYS)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.JWT_SECRET, algorithm=settings.JWT_ALGORITHM)
    return encoded_jwt


def decode_token(token: str) -> dict:
    """Decode and verify JWT token."""
    try:
        payload = jwt.decode(token, settings.JWT_SECRET, algorithms=[settings.JWT_ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired"
        )
    except jwt.JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials"
        )


async def get_current_user(token: str = Depends(oauth2_scheme)) -> dict:
    """Get current user from JWT token."""
    payload = decode_token(token)
    email = payload.get("sub")

    if email is None or email not in USERS_DB:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication credentials"
        )

    return USERS_DB[email]


@router.post("/register", response_model=Token)
async def register(user: UserRegister):
    """
    Register a new user.

    Body:
        {
            "email": "user@example.com",
            "password": "secure_password",
            "name": "John Doe"
        }

    Returns:
        JWT token and user info
    """
    # Check if user already exists
    if user.email in USERS_DB:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Create user
    hashed_password = hash_password(user.password)
    user_data = {
        "email": user.email,
        "name": user.name,
        "hashed_password": hashed_password,
        "created_at": datetime.utcnow()
    }

    USERS_DB[user.email] = user_data

    # Create token
    access_token = create_access_token(data={"sub": user.email})

    return Token(
        access_token=access_token,
        token_type="bearer",
        user={
            "email": user.email,
            "name": user.name,
            "created_at": user_data["created_at"].isoformat()
        }
    )


@router.post("/login", response_model=Token)
async def login(form_data: OAuth2PasswordRequestForm = Depends()):
    """
    Login with email and password.

    Form Data:
        username: Email address
        password: Password

    Returns:
        JWT token and user info
    """
    email = form_data.username  # OAuth2PasswordRequestForm uses 'username' field

    # Check if user exists
    if email not in USERS_DB:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    user_data = USERS_DB[email]

    # Verify password
    if not verify_password(form_data.password, user_data["hashed_password"]):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    # Create token
    access_token = create_access_token(data={"sub": email})

    return Token(
        access_token=access_token,
        token_type="bearer",
        user={
            "email": user_data["email"],
            "name": user_data["name"],
            "created_at": user_data["created_at"].isoformat()
        }
    )


@router.post("/logout")
async def logout(current_user: dict = Depends(get_current_user)):
    """
    Logout user (client should delete token).

    Note: JWT tokens can't be invalidated server-side without a blacklist.
    The client must delete the token from storage.

    Returns:
        Success message
    """
    return {
        "message": "Logged out successfully",
        "email": current_user["email"]
    }


@router.get("/me")
async def get_current_user_info(current_user: dict = Depends(get_current_user)):
    """
    Get current authenticated user info.

    Headers:
        Authorization: Bearer <token>

    Returns:
        User information
    """
    return {
        "email": current_user["email"],
        "name": current_user["name"],
        "created_at": current_user["created_at"].isoformat()
    }


@router.post("/refresh")
async def refresh_token(current_user: dict = Depends(get_current_user)):
    """
    Refresh access token.

    Headers:
        Authorization: Bearer <token>

    Returns:
        New JWT token
    """
    access_token = create_access_token(data={"sub": current_user["email"]})

    return Token(
        access_token=access_token,
        token_type="bearer",
        user={
            "email": current_user["email"],
            "name": current_user["name"],
            "created_at": current_user["created_at"].isoformat()
        }
    )


# OAuth2 endpoints (GitHub, Google) - Placeholder for future implementation
@router.get("/oauth/github")
async def github_oauth():
    """GitHub OAuth login (to be implemented)."""
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="GitHub OAuth not yet implemented"
    )


@router.get("/oauth/google")
async def google_oauth():
    """Google OAuth login (to be implemented)."""
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="Google OAuth not yet implemented"
    )
